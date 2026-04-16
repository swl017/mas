#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <cv_bridge/cv_bridge.h>

#include <atomic>
#include <cstring>
#include <thread>

class RTSPCameraNode : public rclcpp::Node {
public:
    RTSPCameraNode()
    : Node("rtsp_camera_node")
    {
        this->declare_parameter<std::string>("camera_name", "camera");
        this->declare_parameter<std::string>("rtsp_url", "rtsp://192.168.144.25:8554/main.264");
        this->declare_parameter<int>("width", 1280);
        this->declare_parameter<int>("height", 720);
        // latency_ms sets rtspsrc's jitter-buffer ceiling; it is NOT the
        // playback delay when the downstream sink (our appsink here) has
        // sync=false, so the jitter buffer only matters for reordering
        // tolerance. Benched side-by-side (2026-04-16): going from 10 ms
        // drop-on-latency=true to 200 ms drop-on-latency=false cost ≤1 ms
        // median header-age / ≤2 ms p95 at steady state, while avoiding
        // single-packet drops on any link jitter. Keeping the knobs
        // exposed in case a link with drastically different characteristics
        // wants tighter or looser behaviour.
        this->declare_parameter<int>("latency_ms", 200);
        this->declare_parameter<bool>("drop_on_latency", false);
        this->declare_parameter<bool>("use_tcp", false);
        this->declare_parameter<bool>("do_retransmission", false);
        // SIYI A8 mini's main stream is H.265 (verified via rtspsrc caps
        // dump — encoding-name=H265). The ZR10 / older firmwares expose
        // H.264. Override `codec` to swap depay/parse/decoder as a set.
        this->declare_parameter<std::string>("codec", "h265");
        this->declare_parameter<std::string>("decoder", "");
        // Downstream choice: raw BGR, JPEG compressed, or both. Each
        // output is gated on subscriber_count>0 so benchmarks that only
        // consume one topic don't pay for the other (no BGR memcpy, no
        // cv::imencode). With both off the node still runs the pipeline
        // for debug/stream liveness but does not publish.
        this->declare_parameter<bool>("publish_raw", true);
        this->declare_parameter<bool>("publish_compressed", true);
        this->declare_parameter<int>("jpeg_quality", 80);

        this->get_parameter("camera_name", camera_name_);
        this->get_parameter("rtsp_url", rtsp_url_);
        this->get_parameter("width", width_);
        this->get_parameter("height", height_);
        this->get_parameter("latency_ms", latency_ms_);
        this->get_parameter("drop_on_latency", drop_on_latency_);
        this->get_parameter("use_tcp", use_tcp_);
        this->get_parameter("do_retransmission", do_retransmission_);
        this->get_parameter("codec", codec_);
        this->get_parameter("decoder", decoder_name_);
        this->get_parameter("publish_raw", publish_raw_);
        this->get_parameter("publish_compressed", publish_compressed_);
        this->get_parameter("jpeg_quality", jpeg_quality_);
        if (decoder_name_.empty()) {
            decoder_name_ = std::string("avdec_") + codec_;
        }
        depay_name_ = std::string("rtp") + codec_ + "depay";
        parse_name_ = codec_ + "parse";

        // Sensor-stream QoS (BEST_EFFORT, KEEP_LAST, depth=5). Reliable
        // with depth=10 lets a lagging subscriber block the GStreamer
        // streaming thread once the publisher queue fills, which showed
        // up as 500+ ms hitches against the A8's 40 ms frame period.
        auto qos = rclcpp::SensorDataQoS();
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(camera_name_ + "/image_raw", qos);
        compressed_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(camera_name_ + "/image_raw/compressed", qos);

        RCLCPP_INFO(this->get_logger(),
                    "Starting RTSP camera: url=%s codec=%s depay=%s parse=%s decoder=%s latency=%d ms drop=%d tcp=%d",
                    rtsp_url_.c_str(), codec_.c_str(),
                    depay_name_.c_str(), parse_name_.c_str(),
                    decoder_name_.c_str(), latency_ms_,
                    static_cast<int>(drop_on_latency_), static_cast<int>(use_tcp_));

        gst_init(nullptr, nullptr);
        start_pipeline();
    }

    ~RTSPCameraNode()
    {
        running_.store(false);
        if (main_loop_) {
            g_main_loop_quit(main_loop_);
        }
        if (loop_thread_.joinable()) {
            loop_thread_.join();
        }
        if (pipeline_) {
            gst_element_set_state(pipeline_, GST_STATE_NULL);
            gst_object_unref(pipeline_);
        }
        if (main_loop_) {
            g_main_loop_unref(main_loop_);
        }
    }

private:
    void start_pipeline()
    {
        pipeline_ = gst_pipeline_new("rtsp-pipeline");
        src_ = gst_element_factory_make("rtspsrc", "source");
        depay_ = gst_element_factory_make(depay_name_.c_str(), "depay");
        parse_ = gst_element_factory_make(parse_name_.c_str(), "parse");
        decoder_ = gst_element_factory_make(decoder_name_.c_str(), "decoder");
        videoconvert_ = gst_element_factory_make("videoconvert", "videoconvert");
        queue_ = gst_element_factory_make("queue", "leaky_queue");
        appsink_ = gst_element_factory_make("appsink", "appsink");

        // Jetson's `nvv4l2decoder` emits NVMM DMA buffers; they have to be
        // pulled into host memory via `nvvidconv` before the CPU-side
        // `videoconvert` can turn them into BGR. Detect that by decoder
        // name prefix and splice `nvvidconv` in.
        const bool nvmm_path = decoder_name_.rfind("nv", 0) == 0;
        if (nvmm_path) {
            nvvidconv_ = gst_element_factory_make("nvvidconv", "nvvidconv");
        }

        if (!pipeline_ || !src_ || !depay_ || !parse_ || !decoder_ || !videoconvert_ || !queue_ || !appsink_ ||
            (nvmm_path && !nvvidconv_)) {
            RCLCPP_ERROR(this->get_logger(),
                         "Failed to create GStreamer elements (codec=%s depay=%s parse=%s decoder=%s nvvidconv=%d available?)",
                         codec_.c_str(), depay_name_.c_str(),
                         parse_name_.c_str(), decoder_name_.c_str(),
                         static_cast<int>(nvmm_path));
            return;
        }

        g_object_set(G_OBJECT(src_), "location", rtsp_url_.c_str(), NULL);
        g_object_set(G_OBJECT(src_), "latency", latency_ms_, NULL);
        g_object_set(G_OBJECT(src_), "drop-on-latency", drop_on_latency_ ? TRUE : FALSE, NULL);
        g_object_set(G_OBJECT(src_), "do-retransmission", do_retransmission_ ? TRUE : FALSE, NULL);
        // buffer-mode=4 (GST_RTP_JITTER_BUFFER_MODE_SYNCED) is live-friendly.
        g_object_set(G_OBJECT(src_), "buffer-mode", 4, NULL);
        if (use_tcp_) {
            // GST_RTSP_LOWER_TRANS_TCP = (1 << 2) = 0x04; combined UDP is 0x03.
            g_object_set(G_OBJECT(src_), "protocols", 0x04, NULL);
        }

        // Keep SPS/PPS (or VPS/SPS/PPS for H.265) in-band so the decoder
        // can resync on the next IDR.
        g_object_set(G_OBJECT(parse_), "config-interval", -1, NULL);

        // nvv4l2decoder default behavior holds reference frames in the
        // Decoded Picture Buffer, which adds visible latency vs. the
        // software avdec_h264 pipeline even though the header.stamp age
        // stays low (the buffering happens before we stamp). `disable-dpb`
        // is the documented low-latency switch; `enable-max-performance`
        // relaxes throughput-vs-latency throttling in the v4l2 driver.
        // Not all `nv*` decoders expose these, so guard with a property
        // lookup.
        if (nvmm_path) {
            GObjectClass* cls = G_OBJECT_GET_CLASS(decoder_);
            if (g_object_class_find_property(cls, "disable-dpb")) {
                g_object_set(G_OBJECT(decoder_), "disable-dpb", TRUE, NULL);
            }
            if (g_object_class_find_property(cls, "enable-max-performance")) {
                g_object_set(G_OBJECT(decoder_), "enable-max-performance", TRUE, NULL);
            }
        }

        // Drop the oldest frame if the consumer is slow.
        g_object_set(G_OBJECT(queue_),
                     "max-size-buffers", 1u,
                     "max-size-bytes", 0u,
                     "max-size-time", (guint64)0,
                     "leaky", 2,  // GST_QUEUE_LEAK_DOWNSTREAM
                     NULL);

        g_object_set(G_OBJECT(appsink_),
                     "emit-signals", TRUE,
                     "sync", FALSE,
                     "async", FALSE,
                     "max-buffers", 1u,
                     "drop", TRUE,
                     NULL);
        GstCaps* caps = gst_caps_from_string("video/x-raw, format=(string)BGR");
        gst_app_sink_set_caps(GST_APP_SINK(appsink_), caps);
        gst_caps_unref(caps);

        gst_bin_add_many(GST_BIN(pipeline_), src_, depay_, parse_, decoder_, videoconvert_, queue_, appsink_, NULL);
        if (nvvidconv_) {
            gst_bin_add(GST_BIN(pipeline_), nvvidconv_);
        }

        const bool linked = nvvidconv_
            ? gst_element_link_many(depay_, parse_, decoder_, nvvidconv_, videoconvert_, queue_, appsink_, NULL)
            : gst_element_link_many(depay_, parse_, decoder_, videoconvert_, queue_, appsink_, NULL);
        if (!linked) {
            RCLCPP_ERROR(this->get_logger(), "Failed to link elements.");
            gst_object_unref(pipeline_);
            pipeline_ = nullptr;
            return;
        }

        // rtspsrc creates its video pad dynamically; hook it to the depay on pad-added.
        g_signal_connect(src_, "pad-added",
                         G_CALLBACK(+[](GstElement*, GstPad* new_pad, gpointer user_data) {
                             GstElement* depay = GST_ELEMENT(user_data);
                             GstPad* sink_pad = gst_element_get_static_pad(depay, "sink");
                             if (!gst_pad_is_linked(sink_pad)) {
                                 gst_pad_link(new_pad, sink_pad);
                             }
                             gst_object_unref(sink_pad);
                         }),
                         depay_);

        // Sample delivery callback — runs synchronously as soon as GStreamer posts a frame.
        g_signal_connect(appsink_, "new-sample",
                         G_CALLBACK(+[](GstElement* sink, gpointer user_data) -> GstFlowReturn {
                             auto* self = static_cast<RTSPCameraNode*>(user_data);
                             GstSample* sample = gst_app_sink_pull_sample(GST_APP_SINK(sink));
                             if (sample) {
                                 self->handle_sample(sample);
                                 gst_sample_unref(sample);
                             }
                             return GST_FLOW_OK;
                         }),
                         this);

        // Bus watch for error / EOS on the GLib main loop.
        bus_ = gst_pipeline_get_bus(GST_PIPELINE(pipeline_));
        gst_bus_add_watch(bus_,
                          +[](GstBus*, GstMessage* msg, gpointer user_data) -> gboolean {
                              auto* self = static_cast<RTSPCameraNode*>(user_data);
                              switch (GST_MESSAGE_TYPE(msg)) {
                                  case GST_MESSAGE_ERROR: {
                                      GError* err = nullptr;
                                      gchar* debug_info = nullptr;
                                      gst_message_parse_error(msg, &err, &debug_info);
                                      RCLCPP_ERROR(self->get_logger(), "Pipeline error from %s: %s (%s)",
                                                   GST_OBJECT_NAME(msg->src),
                                                   err ? err->message : "?",
                                                   debug_info ? debug_info : "");
                                      g_clear_error(&err);
                                      g_free(debug_info);
                                      g_main_loop_quit(self->main_loop_);
                                      break;
                                  }
                                  case GST_MESSAGE_EOS:
                                      RCLCPP_INFO(self->get_logger(), "EOS received.");
                                      g_main_loop_quit(self->main_loop_);
                                      break;
                                  default:
                                      break;
                              }
                              return TRUE;
                          },
                          this);

        gst_element_set_state(pipeline_, GST_STATE_PLAYING);

        // GLib main loop services the new-sample callback + bus watch.
        main_loop_ = g_main_loop_new(nullptr, FALSE);
        loop_thread_ = std::thread([this]() {
            g_main_loop_run(main_loop_);
        });
    }

    void handle_sample(GstSample* sample)
    {
        GstBuffer* buffer = gst_sample_get_buffer(sample);
        GstCaps* caps = gst_sample_get_caps(sample);
        if (!buffer || !caps) {
            return;
        }
        GstStructure* s = gst_caps_get_structure(caps, 0);
        int stream_w = 0, stream_h = 0;
        gst_structure_get_int(s, "width", &stream_w);
        gst_structure_get_int(s, "height", &stream_h);

        GstMapInfo map;
        if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {
            return;
        }

        const rclcpp::Time stamp = this->get_clock()->now();
        const bool needs_resize = (stream_w != width_ || stream_h != height_);

        // Compute (out_ptr, out_w, out_h) once; resize lazily into
        // `resized_` scratch only if requested dims differ from stream.
        int out_w = stream_w;
        int out_h = stream_h;
        const uint8_t* out_ptr = map.data;
        if (needs_resize) {
            cv::Mat src(cv::Size(stream_w, stream_h), CV_8UC3, map.data, cv::Mat::AUTO_STEP);
            cv::resize(src, resized_, cv::Size(width_, height_));
            out_w = resized_.cols;
            out_h = resized_.rows;
            out_ptr = resized_.data;
        }

        const bool have_raw_sub = publish_raw_ && image_pub_->get_subscription_count() > 0;
        const bool have_cmp_sub = publish_compressed_ && compressed_pub_->get_subscription_count() > 0;

        if (have_raw_sub) {
            auto msg = std::make_unique<sensor_msgs::msg::Image>();
            msg->header.stamp = stamp;
            msg->header.frame_id = camera_name_;
            msg->encoding = "bgr8";
            msg->is_bigendian = false;
            msg->width = out_w;
            msg->height = out_h;
            msg->step = 3 * out_w;
            msg->data.resize(msg->step * msg->height);
            std::memcpy(msg->data.data(), out_ptr, msg->data.size());
            image_pub_->publish(std::move(msg));
        }

        if (have_cmp_sub) {
            auto cmsg = std::make_unique<sensor_msgs::msg::CompressedImage>();
            cmsg->header.stamp = stamp;
            cmsg->header.frame_id = camera_name_;
            cmsg->format = "jpeg";
            cv::Mat bgr(out_h, out_w, CV_8UC3, const_cast<uint8_t*>(out_ptr));
            const std::vector<int> enc_params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
            cv::imencode(".jpg", bgr, cmsg->data, enc_params);
            compressed_pub_->publish(std::move(cmsg));
        }

        gst_buffer_unmap(buffer, &map);
    }

    std::string camera_name_;
    std::string rtsp_url_;
    int width_;
    int height_;
    int latency_ms_;
    bool drop_on_latency_;
    bool use_tcp_;
    bool do_retransmission_;
    std::string codec_;
    std::string decoder_name_;
    std::string depay_name_;
    std::string parse_name_;
    bool publish_raw_ = true;
    bool publish_compressed_ = true;
    int jpeg_quality_ = 80;
    cv::Mat resized_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_pub_;

    GstElement* pipeline_ = nullptr;
    GstElement* src_ = nullptr;
    GstElement* depay_ = nullptr;
    GstElement* parse_ = nullptr;
    GstElement* decoder_ = nullptr;
    GstElement* nvvidconv_ = nullptr;
    GstElement* videoconvert_ = nullptr;
    GstElement* queue_ = nullptr;
    GstElement* appsink_ = nullptr;
    GstBus* bus_ = nullptr;
    GMainLoop* main_loop_ = nullptr;

    std::thread loop_thread_;
    std::atomic<bool> running_{true};
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RTSPCameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
