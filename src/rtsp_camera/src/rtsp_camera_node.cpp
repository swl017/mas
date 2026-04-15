#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
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
        this->declare_parameter<int>("latency_ms", 10);
        this->declare_parameter<bool>("drop_on_latency", true);
        this->declare_parameter<bool>("use_tcp", false);
        this->declare_parameter<bool>("do_retransmission", false);
        this->declare_parameter<std::string>("decoder", "avdec_h264");

        this->get_parameter("camera_name", camera_name_);
        this->get_parameter("rtsp_url", rtsp_url_);
        this->get_parameter("width", width_);
        this->get_parameter("height", height_);
        this->get_parameter("latency_ms", latency_ms_);
        this->get_parameter("drop_on_latency", drop_on_latency_);
        this->get_parameter("use_tcp", use_tcp_);
        this->get_parameter("do_retransmission", do_retransmission_);
        this->get_parameter("decoder", decoder_name_);

        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(camera_name_ + "/image_raw", 10);

        RCLCPP_INFO(this->get_logger(),
                    "Starting RTSP camera: url=%s decoder=%s latency=%d ms drop=%d tcp=%d",
                    rtsp_url_.c_str(), decoder_name_.c_str(), latency_ms_,
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
        depay_ = gst_element_factory_make("rtph264depay", "depay");
        h264parse_ = gst_element_factory_make("h264parse", "h264parse");
        decoder_ = gst_element_factory_make(decoder_name_.c_str(), "decoder");
        videoconvert_ = gst_element_factory_make("videoconvert", "videoconvert");
        queue_ = gst_element_factory_make("queue", "leaky_queue");
        appsink_ = gst_element_factory_make("appsink", "appsink");

        if (!pipeline_ || !src_ || !depay_ || !h264parse_ || !decoder_ || !videoconvert_ || !queue_ || !appsink_) {
            RCLCPP_ERROR(this->get_logger(),
                         "Failed to create GStreamer elements (decoder=%s available?)",
                         decoder_name_.c_str());
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

        // Keep SPS/PPS in-band so the decoder can resync on the next IDR.
        g_object_set(G_OBJECT(h264parse_), "config-interval", -1, NULL);

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

        gst_bin_add_many(GST_BIN(pipeline_), src_, depay_, h264parse_, decoder_, videoconvert_, queue_, appsink_, NULL);

        if (!gst_element_link_many(depay_, h264parse_, decoder_, videoconvert_, queue_, appsink_, NULL)) {
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

        auto msg = std::make_unique<sensor_msgs::msg::Image>();
        msg->header.stamp = this->get_clock()->now();
        msg->header.frame_id = camera_name_;
        msg->encoding = "bgr8";
        msg->is_bigendian = false;

        const bool needs_resize = (stream_w != width_ || stream_h != height_);
        if (!needs_resize) {
            msg->width = stream_w;
            msg->height = stream_h;
            msg->step = 3 * stream_w;
            msg->data.resize(msg->step * msg->height);
            std::memcpy(msg->data.data(), map.data, msg->data.size());
        } else {
            cv::Mat frame(cv::Size(stream_w, stream_h), CV_8UC3, map.data, cv::Mat::AUTO_STEP);
            cv::Mat resized;
            cv::resize(frame, resized, cv::Size(width_, height_));
            msg->width = resized.cols;
            msg->height = resized.rows;
            msg->step = 3 * resized.cols;
            msg->data.resize(msg->step * msg->height);
            std::memcpy(msg->data.data(), resized.data, msg->data.size());
        }

        image_pub_->publish(std::move(msg));
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
    std::string decoder_name_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

    GstElement* pipeline_ = nullptr;
    GstElement* src_ = nullptr;
    GstElement* depay_ = nullptr;
    GstElement* h264parse_ = nullptr;
    GstElement* decoder_ = nullptr;
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
