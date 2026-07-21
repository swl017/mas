/**
 * @file replay_coop_smoother.cpp
 * @brief RAL ticket 024 pre-S4 (rev2 gate) — ARRIVAL-ORDER offline replay of the cooperative
 *        smoother on a captured (instrumented) engagement, re-running the PRODUCTION CoopSmoother.
 *
 * v2 (supersedes the capture-time-availability v1, archived in the RAL 024 ticket): this tool is
 * an event machine that emulates coop_smoother_node's callback semantics record-for-record on the
 * v2 flat stream from s3_replay_extract.py (rosbag record order = arrival-order proxy):
 *   - K/G/Z/O events update have_K_/K_raw_, have_gimbal_/gimbal_rad_, zoom_, pose_buf_ (cap 200)
 *     exactly as the node's callbacks do — an ego detection is assembled with the LAST-ARRIVED
 *     gimbal/zoom and the pose buffer as of its arrival (not last-by-capture-stamp, the v1 flaw);
 *   - D/P events append to a measurement deque in arrival order (onDetection/onPeerRays),
 *     including the node's skip gates (no K / no gimbal / empty or non-bracketing pose buffer);
 *   - X events (the recorded live solver_diagnostics publishes) fire onTimer(): front-only window
 *     prune, fresh CoopSmoother fed in buffer order, solve, query(now) — and the live X fields are
 *     written next to the replay's so scheduling fidelity is measurable per tick.
 * Outputs per tick: replay + live diagnostics, belief p/v, full 3x3 position + velocity covariance
 * blocks (forward-predicted, as published), recorded live belief + its published position
 * covariance, GT position AND velocity, errors, and NEES (replay + recorded-live) — the S4
 * ANEES/coverage gate inputs, maskable by n_peer (acquisition vs engaged).
 *
 * Build: colcon target replay_coop_smoother. Run:
 *   replay_coop_smoother <in.v2.txt> <out.csv> [pixel_sigma=2.0] [bearing_deg=0.5]
 *       [sigma_psi_deg=0.0] [use_robust=0] [window_s=0.6] [q_c=4.0] [gimbal_order=zxy]
 *       [ego_audit_csv|-] [det_lag_s=0] [peer_att_deg=0] [peer_pos_m=0]
 *       [vel_cov_inflation=1] [use_robust_ego=0] [gate_enabled=1] [warm_start=0] [warm_max_age_s=2]
 *       [backend=0|1]   (0 batch full-window, 1 fixed-lag/iSAM2)
 *   (out.csv = "-" -> audit-only mode, no solves)
 *
 * S4 ego-audit mode (optional 10th arg): per detection event, evaluate the PRODUCTION
 * EgoPixelFactor residual (observed - projected) at the GT target position interpolated to
 * t_det — the empirical ego measurement error in the factor's own units (pixels), through the
 * byte-identical camera assembly + projection. Feeds the S4 sigma characterization.
 */
#include "coop_smoother.h"
#include "coop_smoother_fl.h"
#include "ego_camera.h"
#include "meas_noise.h"
#include "output_gate.h"
#include "pose_interp.h"

#include <memory>

#include <Eigen/Eigen>
#include <algorithm>
#include <cmath>
#include <deque>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using mas_fgo::CoopSmoother;

namespace {
enum MType { EGO_PIXEL, PEER_BEARING };
struct Meas {
    double t; MType type;
    Eigen::Vector2d px; Eigen::Matrix3d K, R; Eigen::Vector3d cam_t;  // ego
    Eigen::Vector3d o, d;                                             // peer
};
struct State { double t; Eigen::Vector3d p, v; };
struct RecC {  // recorded live belief + published 3x3 position covariance
    double ta, t; Eigen::Vector3d p, v; double c[6];
};
struct Ev {    // arrival-ordered event
    double ta; char tag; std::vector<double> num; std::string s1, s2;
};

Eigen::Vector3d interp_state(const std::vector<State>& s, double t, bool vel) {
    if (s.empty()) return Eigen::Vector3d::Zero();
    if (t <= s.front().t) return vel ? s.front().v : s.front().p;
    if (t >= s.back().t) return vel ? s.back().v : s.back().p;
    int lo = 0, hi = (int)s.size() - 1;
    while (hi - lo > 1) { int m = (lo + hi) / 2; (s[m].t <= t ? lo : hi) = m; }
    const double a = (t - s[lo].t) / std::max(1e-9, s[hi].t - s[lo].t);
    const Eigen::Vector3d& A = vel ? s[lo].v : s[lo].p;
    const Eigen::Vector3d& B = vel ? s[hi].v : s[hi].p;
    return A + a * (B - A);
}

double nees3(const Eigen::Vector3d& e, const Eigen::Matrix3d& P) {
    const double det = P.determinant();
    if (!std::isfinite(det) || det < 1e-30) return std::nan("");
    return e.dot(P.inverse() * e);
}

Eigen::Matrix3d sym3(const double c[6]) {
    Eigen::Matrix3d M;
    M << c[0], c[1], c[2],
         c[1], c[3], c[4],
         c[2], c[4], c[5];
    return M;
}
}  // namespace

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "usage: replay_coop_smoother <in.v2.txt> <out.csv> [pixel_sigma] [bearing_deg] "
                     "[sigma_psi_deg] [use_robust] [window_s] [q_c] [gimbal_order]\n";
        return 1;
    }
    const std::string in = argv[1], out = argv[2];
    const double pixel_sigma = argc > 3 ? std::stod(argv[3]) : 2.0;
    const double bearing_deg = argc > 4 ? std::stod(argv[4]) : 0.5;
    const double sigma_psi_deg = argc > 5 ? std::stod(argv[5]) : 0.0;
    const bool use_robust = argc > 6 ? std::stoi(argv[6]) != 0 : false;
    const double window_s = argc > 7 ? std::stod(argv[7]) : 0.6;
    const double q_c = argc > 8 ? std::stod(argv[8]) : 4.0;
    const std::string order = argc > 9 ? argv[9] : "zxy";
    std::ofstream audit;
    if (argc > 10 && std::string(argv[10]) != "-") {
        audit.open(argv[10]);
        audit << "t_det,u,v,res_u,res_v,w,h,score,fx,range_gt\n";
    }
    // S4 detection-latency hypothesis: the detection stamp lags the true frame capture by
    // det_lag_s; the effective capture time t_eff = t_stamp - det_lag_s is used for pose
    // interpolation, factor placement, and the audit's GT lookup.
    // (Measured: REFUTED for these captures — v-scatter explodes with any shift, so the
    // recorded pose<->frame pairing is already correct; keep 0 unless re-diagnosing.)
    const double det_lag = argc > 11 ? std::stod(argv[11]) : 0.0;
    // S4 Q9 characterized peer constants (deployment-grade fallback for the not-yet-transmitted
    // EKF2 covariance): isotropic attitude sigma [deg] and origin/position sigma [m]; >0 enables
    // the corresponding buildPeerBearingCov term.
    const double peer_att_deg = argc > 12 ? std::stod(argv[12]) : 0.0;
    const double peer_pos_m = argc > 13 ? std::stod(argv[13]) : 0.0;
    // S5: declared vel-cov inflation, ego robust kernel, output gate (production output_gate.h;
    // both the raw solver belief and the gated published stream are emitted per tick).
    const double vel_infl = argc > 14 ? std::stod(argv[14]) : 1.0;
    const bool robust_ego = argc > 15 ? std::stoi(argv[15]) != 0 : false;
    const bool gate_on = argc > 16 ? std::stoi(argv[16]) != 0 : true;
    // S6: warm-start from the last gate-accepted belief (CV-propagated), age-limited.
    const bool warm_on = argc > 17 ? std::stoi(argv[17]) != 0 : false;
    const double warm_max_age = argc > 18 ? std::stod(argv[18]) : 2.0;
    // S7: backend 0 = batch full-window (default), 1 = fixed-lag/iSAM2 (persistent).
    const int backend = argc > 19 ? std::stoi(argv[19]) : 0;
    // S7 finite-memory decay for the FL prior (0 = off).
    const double fl_reset_s = argc > 20 ? std::stod(argv[20]) : 0.0;
    // RAL ticket 028 (ego-only backend-matched control):
    //   gate_min_peer : output-gate min_peer (1 = 024 behavior; 0 = ego-only mode)
    //   init_range_m  : acquisition fallback range along the first ray (BO-EKF prior parity)
    //   tick_mode     : 0 = tick at X records (024 default); 1 = tick at C-record stamps —
    //                   ego bags have no live smoother (no X records); their C stream is the
    //                   recorded ego-EKF belief, so both backends are queried at identical
    //                   instants (pairing by construction)
    //   ray_policy    : 0 = all rays (default); 1 = endpoint-K; 2 = decimate keep-every-j,
    //                   newest kept (S2b ablation; ego factors only; batch backend only)
    //   ray_param     : K for endpoint-K, j for decimate
    const int gate_min_peer = argc > 21 ? std::stoi(argv[21]) : 1;
    const double init_range_m = argc > 22 ? std::stod(argv[22]) : 30.0;
    const int tick_mode = argc > 23 ? std::stoi(argv[23]) : 0;
    const int ray_policy = argc > 24 ? std::stoi(argv[24]) : 0;
    const int ray_param = argc > 25 ? std::stoi(argv[25]) : 0;
    if (ray_policy != 0 && backend == 1) {
        std::cerr << "ray_policy requires the batch backend (backend=0)\n";
        return 1;
    }
    // RAL ticket 028 S2c remedy probe (batch backend only):
    //   use_bias         : shared 2-DOF pixel-bias state per solve (EgoPixelBiasFactor)
    //   bias_sigma_px    : zero-mean prior sigma on the bias state
    //   range_mem_sigma_m: >0 arms the A4 depth-memory range prior on the newest state,
    //                      centered at the last RAW SOLVED belief's CV-propagated range
    //                      (rev1 B6: adaptation from the i_design_s2c gate-accepted
    //                      source — that stream is empty on ego bags; exploratory)
    const bool use_bias = argc > 26 ? std::stoi(argv[26]) != 0 : false;
    const double bias_sigma_px = argc > 27 ? std::stod(argv[27]) : 200.0;
    const double range_mem_sigma = argc > 28 ? std::stod(argv[28]) : 0.0;
    if ((use_bias || range_mem_sigma > 0.0) && backend == 1) {
        std::cerr << "S2c arms (use_bias / range_mem) require the batch backend (backend=0)\n";
        return 1;
    }

    // ---- parse the v2 stream (file order = arrival order) ----
    std::ifstream f(in);
    if (!f) { std::cerr << "cannot open " << in << "\n"; return 1; }
    std::string line;
    if (!std::getline(f, line) || line.rfind("V 2", 0) != 0) {
        std::cerr << "input is not a v2 stream (missing 'V 2' header) — re-extract with the v2 "
                     "s3_replay_extract.py\n";
        return 1;
    }
    std::vector<Ev> evs;
    std::vector<State> Ts;
    std::vector<RecC> Cs;
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        std::istringstream ss(line);
        std::string tag; ss >> tag;
        if (tag == "T") {
            double t, px, py, pz, vx, vy, vz; ss >> t >> px >> py >> pz >> vx >> vy >> vz;
            Ts.push_back({t, {px, py, pz}, {vx, vy, vz}});
        } else if (tag == "C") {
            RecC c; double px, py, pz, vx, vy, vz;
            ss >> c.ta >> c.t >> px >> py >> pz >> vx >> vy >> vz
               >> c.c[0] >> c.c[1] >> c.c[2] >> c.c[3] >> c.c[4] >> c.c[5];
            c.p = {px, py, pz}; c.v = {vx, vy, vz};
            Cs.push_back(c);
            if (tick_mode == 1) {  // RAL ticket 028: query tick at the recorded belief stamp
                Ev e2; e2.tag = 'Q'; e2.ta = c.ta; e2.num.push_back(c.t);
                evs.push_back(std::move(e2));
            }
        } else {
            Ev e; e.tag = tag[0]; ss >> e.ta;
            double v;
            if (tag == "D") {  // ta t px py w h score cls id idx n
                for (int i = 0; i < 6; ++i) { ss >> v; e.num.push_back(v); }
                ss >> e.s1 >> e.s2;
                for (int i = 0; i < 2; ++i) { ss >> v; e.num.push_back(v); }
            } else if (tag == "P") {  // ta t ox oy oz dx dy dz idx n det_id
                for (int i = 0; i < 9; ++i) { ss >> v; e.num.push_back(v); }
                ss >> e.s1;
            } else {
                while (ss >> v) e.num.push_back(v);
            }
            evs.push_back(std::move(e));
        }
    }
    std::sort(Ts.begin(), Ts.end(), [](auto& a, auto& b) { return a.t < b.t; });
    std::cerr << "parsed: " << evs.size() << " events, C=" << Cs.size() << " T=" << Ts.size() << "\n";

    // ---- node-state emulation ----
    Eigen::Matrix3d K_raw = Eigen::Matrix3d::Identity();
    bool have_K = false, have_gimbal = false;
    Eigen::Vector3d gimbal_rad = Eigen::Vector3d::Zero();
    double zoom = 1.0;
    std::deque<mas_fgo::PoseSample> pose_buf;
    std::deque<Meas> buf;
    int drop_gate = 0, drop_interp = 0;

    mas_fgo::CoopSmoother::Params prm;
    prm.window_s = window_s; prm.q_c = q_c; prm.use_robust = use_robust;
    prm.use_robust_ego = robust_ego; prm.vel_cov_inflation = vel_infl;
    prm.init_range_m = init_range_m;                        // RAL ticket 028
    prm.use_bias_state = use_bias;                          // RAL ticket 028 S2c
    prm.bias_sigma_px = bias_sigma_px;
    mas_fgo::GateParams gp; gp.q_c = q_c;
    gp.min_peer = gate_min_peer;                            // RAL ticket 028
    mas_fgo::GateState gst;
    std::string last_track_id;
    std::vector<Ev> pend;  // detections of the current array (consecutive D records)
    std::unique_ptr<mas_fgo::CoopSmootherFL> fl;
    if (backend == 1) fl = std::make_unique<mas_fgo::CoopSmootherFL>(prm, fl_reset_s);
    mas_fgo::PeerNoiseParams np;
    np.sigma_static_rad = bearing_deg * M_PI / 180.0;
    np.sigma_psi_rad = sigma_psi_deg * M_PI / 180.0;
    np.include_attitude = peer_att_deg > 0.0; np.include_origin = peer_pos_m > 0.0;
    const double att_var = std::pow(peer_att_deg * M_PI / 180.0, 2);
    const Eigen::Matrix3d Sig_att = Eigen::Matrix3d::Identity() * att_var;
    const Eigen::Matrix3d Sig_pos = Eigen::Matrix3d::Identity() * (peer_pos_m * peer_pos_m);

    struct Tick {
        double now;
        mas_fgo::CoopSmoother::Diagnostics rd; bool r_solved = false; int r_buf = 0;
        std::vector<double> live;                 // the 16 X fields
        mas_fgo::CoopSmoother::Query q;           // replay belief (q.valid=false if unsolved)
        mas_fgo::GateOutput go;                   // S5 gated published stream
    };
    std::vector<Tick> ticks;

    // RAL 028 S2c (A4): rolling last-raw-solve memory for the depth prior.
    bool rmem_have = false; double rmem_t = 0.0;
    Eigen::Vector3d rmem_p = Eigen::Vector3d::Zero(), rmem_v = Eigen::Vector3d::Zero();

    // RAL ticket 028: shared tick body — fired at X records (tick_mode=0, the 024 path,
    // live diagnostics attached) or at C-record stamps (tick_mode=1, Q events, no live).
    auto run_tick = [&](double now, const std::vector<double>& live) {
        Tick tk; tk.now = now; tk.live = live;
        if (fl) {  // S7 fixed-lag: persistent update, no rebuild
            tk.r_solved = fl->update(tk.now);
            tk.rd = fl->diagnostics();
            if (tk.r_solved) tk.q = fl->query(tk.now);
            if (gate_on) {
                const Eigen::Vector3d ego =
                    pose_buf.empty() ? Eigen::Vector3d::Zero() : pose_buf.back().p;
                tk.go = mas_fgo::applyOutputGate(tk.rd, tk.r_solved, tk.q, tk.now, ego,
                                                 !pose_buf.empty(), gp, gst);
            }
            ticks.push_back(std::move(tk));
            return;
        }
        const double t_cut = tk.now - window_s - 0.2;
        while (!buf.empty() && buf.front().t < t_cut) buf.pop_front();
        tk.r_buf = (int)buf.size();
        if (buf.size() >= 2) {
            mas_fgo::CoopSmoother sm(prm);
            if (warm_on && gst.have_last && tk.now - gst.t_last >= 0.0 &&
                tk.now - gst.t_last <= warm_max_age) {
                sm.setInitHint(gst.t_last, gst.p_last, gst.v_last);
            }
            // RAL 028 S2c (A4): depth-memory prior from the last RAW solved belief —
            // deliberate self-feedback (i_design_s2c E4); the gate-accepted stream is
            // empty on ego bags, so gst cannot be the source.
            if (range_mem_sigma > 0.0 && rmem_have && !pose_buf.empty()) {
                const double dt_m = tk.now - rmem_t;
                if (dt_m >= 0.0 && dt_m <= warm_max_age) {
                    const Eigen::Vector3d pm = rmem_p + rmem_v * dt_m;
                    sm.setRangeMemory(pose_buf.back().p,
                                      (pm - pose_buf.back().p).norm(), range_mem_sigma);
                }
            }
            auto add_meas = [&](const Meas& m) {
                if (m.type == EGO_PIXEL) {
                    sm.addEgoPixel(m.t, m.px, m.K, m.R, m.cam_t, pixel_sigma);
                } else {
                    const Eigen::Vector3d seed =
                        (std::abs(m.d.x()) < 0.9) ? Eigen::Vector3d::UnitX()
                                                  : Eigen::Vector3d::UnitY();
                    const Eigen::Vector3d u1 = (seed - seed.dot(m.d) * m.d).normalized();
                    const Eigen::Vector3d u2 = m.d.cross(u1);
                    const Eigen::Matrix2d R = mas_fgo::buildPeerBearingCov(
                        np, u1, u2, m.o, m.d, m.o + 30.0 * m.d, Sig_att, Sig_pos);
                    sm.addPeerBearing(m.t, m.o, m.d, R);
                }
            };
            int n_fed = 0;
            if (ray_policy == 0) {
                for (const auto& m : buf) { add_meas(m); ++n_fed; }
            } else {
                // S2b ray-usage policy: subsample EGO factors only (peers always fed).
                std::vector<const Meas*> egos;
                for (const auto& m : buf) {
                    if (m.type == EGO_PIXEL) egos.push_back(&m);
                    else { add_meas(m); ++n_fed; }
                }
                const int ne = (int)egos.size();
                for (int i = 0; i < ne; ++i) {
                    bool keep = true;
                    if (ray_policy == 1 && ray_param > 0 && ne > ray_param) {
                        const int h = ray_param / 2;   // endpoint-K: oldest h + newest K-h
                        keep = i < h || i >= ne - (ray_param - h);
                    } else if (ray_policy == 2 && ray_param > 1) {
                        keep = ((ne - 1 - i) % ray_param) == 0;   // newest always kept
                    }
                    if (keep) { add_meas(*egos[i]); ++n_fed; }
                }
            }
            if (n_fed >= 2) {
                tk.r_solved = sm.solve();
                tk.rd = sm.diagnostics();
                if (tk.r_solved) {
                    tk.q = sm.query(tk.now);
                    rmem_have = true; rmem_t = tk.now;   // S2c A4 memory source
                    rmem_p = tk.q.p; rmem_v = tk.q.v;
                }
            }
        }
        if (gate_on) {
            const Eigen::Vector3d ego =
                pose_buf.empty() ? Eigen::Vector3d::Zero() : pose_buf.back().p;
            tk.go = mas_fgo::applyOutputGate(tk.rd, tk.r_solved, tk.q, tk.now, ego,
                                             !pose_buf.empty(), gp, gst);
        }
        ticks.push_back(std::move(tk));
    };

    for (const auto& e : evs) {
        switch (e.tag) {
        case 'K':
            K_raw.setIdentity();
            K_raw(0, 0) = e.num[0]; K_raw(1, 1) = e.num[1];
            K_raw(0, 2) = e.num[2]; K_raw(1, 2) = e.num[3];
            have_K = true;
            break;
        case 'O': {
            mas_fgo::PoseSample s; s.t = e.num[0];
            s.p = Eigen::Vector3d(e.num[1], e.num[2], e.num[3]);
            s.q = Eigen::Quaterniond(e.num[4], e.num[5], e.num[6], e.num[7]).normalized();
            pose_buf.push_back(s);
            while (pose_buf.size() > 200) pose_buf.pop_front();
            break;
        }
        case 'G':
            gimbal_rad = Eigen::Vector3d(e.num[0], e.num[1], e.num[2]) * M_PI / 180.0;
            have_gimbal = true;
            break;
        case 'Z':
            zoom = e.num[0];
            break;
        case 'D': {  // onDetection: group the array (consecutive D records), then select ONE
            pend.push_back(e);
            if ((int)e.num[6] != (int)e.num[7] - 1) break;  // not the array's last detection
            std::vector<Ev> arr; arr.swap(pend);
            if (!have_K || !have_gimbal || pose_buf.empty()) { ++drop_gate; break; }
            const double t_det = arr[0].num[0] - det_lag;
            const mas_fgo::InterpPose ip = mas_fgo::interpolatePose(pose_buf, t_det);
            if (!ip.valid) { ++drop_interp; break; }
            const mas_fgo::EgoCamera cam =
                mas_fgo::assembleEgoCamera(ip.p, ip.q, gimbal_rad, zoom, K_raw, order);
            if (audit.is_open() && !Ts.empty()) {  // characterization: audit ALL detections
                const Eigen::Vector3d gt = interp_state(Ts, t_det, false);
                for (const auto& d : arr) {
                    const Eigen::Vector2d px(d.num[1], d.num[2]);
                    mas_fgo::EgoPixelFactor f(gtsam::Symbol('x', 0).key(), px, cam.K, cam.R,
                                              cam.t, gtsam::noiseModel::Isotropic::Sigma(2, 1.0));
                    const gtsam::Vector r = f.evaluateError(gtsam::Point3(gt));
                    audit << t_det << "," << px.x() << "," << px.y() << ","
                          << r(0) << "," << r(1) << "," << d.num[3] << "," << d.num[4] << ","
                          << d.num[5] << "," << cam.K(0, 0) << ","
                          << (gt - cam.t).norm() << "\n";
                }
            }
            // S5 association (mirrors the node): class/score filter, prefer last track id,
            // else highest score. Defaults ("", 0.0) are behavior-preserving on 1-det arrays.
            const Ev* best = nullptr; double best_score = -1.0; bool best_track = false;
            for (const auto& d : arr) {
                const double score = d.num[5];
                if (score < 0.0) continue;
                const bool is_track = !last_track_id.empty() && d.s2 == last_track_id;
                if ((is_track && !best_track) || (is_track == best_track && score > best_score)) {
                    best = &d; best_score = score; best_track = is_track;
                }
            }
            if (!best) break;
            last_track_id = best->s2;
            Meas m; m.t = t_det; m.type = EGO_PIXEL;
            m.px = Eigen::Vector2d(best->num[1], best->num[2]);
            m.K = cam.K; m.R = cam.R; m.cam_t = cam.t;
            if (fl) fl->addEgoPixel(m.t, m.px, m.K, m.R, m.cam_t, pixel_sigma);
            else buf.push_back(m);
            break;
        }
        case 'P': {  // onPeerRays
            Eigen::Vector3d d(e.num[4], e.num[5], e.num[6]);
            if (d.norm() < 1e-9) break;
            Meas m; m.t = e.num[0]; m.type = PEER_BEARING;
            m.o = Eigen::Vector3d(e.num[1], e.num[2], e.num[3]); m.d = d.normalized();
            if (fl) {
                const Eigen::Vector3d seed =
                    (std::abs(m.d.x()) < 0.9) ? Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitY();
                const Eigen::Vector3d u1 = (seed - seed.dot(m.d) * m.d).normalized();
                const Eigen::Vector3d u2 = m.d.cross(u1);
                const Eigen::Matrix2d Rn = mas_fgo::buildPeerBearingCov(
                    np, u1, u2, m.o, m.d, m.o + 30.0 * m.d, Sig_att, Sig_pos);
                fl->addPeerBearing(m.t, m.o, m.d, Rn);
            } else {
                buf.push_back(m);
            }
            break;
        }
        case 'X': {  // onTimer at the recorded live tick (tick_mode=0)
            if (out == "-") break;  // audit-only mode: no solves
            if (tick_mode != 0) break;   // RAL ticket 028: C-stamp ticks selected instead
            run_tick(e.ta, e.num);
            break;
        }
        case 'Q': {  // RAL ticket 028: tick at the recorded belief stamp (tick_mode=1)
            if (out == "-") break;
            if (tick_mode != 1) break;
            run_tick(e.num[0], {});
            break;
        }
        default:
            break;
        }
    }

    // ---- join ticks with the recorded live belief (nearest published stamp) ----
    std::vector<double> ct(Cs.size());
    for (size_t i = 0; i < Cs.size(); ++i) ct[i] = Cs[i].t;

    std::ofstream o(out == "-" ? "/dev/null" : out.c_str());
    o << "t,"
         "r_solved,r_iters,r_nkf,r_nego,r_npeer,r_err_before,r_err_after,r_maxfac,"
         "r_seed_x,r_seed_y,r_seed_z,r_buf,"
         "l_solved,l_iters,l_maxiter,l_nkf,l_nego,l_npeer,l_err_before,l_err_after,l_maxfac,"
         "l_toldest,l_tnewest,l_span,l_seed_x,l_seed_y,l_seed_z,l_buf,"
         "bel_x,bel_y,bel_z,bel_vx,bel_vy,bel_vz,"
         "Pp00,Pp01,Pp02,Pp11,Pp12,Pp22,Pv00,Pv01,Pv02,Pv11,Pv12,Pv22,"
         "rec_x,rec_y,rec_z,rec_vx,rec_vy,rec_vz,rec_c00,rec_c01,rec_c02,rec_c11,rec_c12,rec_c22,"
         "gt_x,gt_y,gt_z,gt_vx,gt_vy,gt_vz,"
         "r_perr,r_verr,rec_perr,rec_verr,r_vs_rec,nees_p,nees_v,rec_nees_p,"
         "pub_valid,gate_reason,pub_x,pub_y,pub_z,pub_vx,pub_vy,pub_vz,"
         "pub_perr,pub_verr,pub_nees_p\n";

    int n_solved = 0, n_diag_match = 0, n_have_c = 0;
    for (const auto& tk : ticks) {
        const Eigen::Vector3d gt_p = interp_state(Ts, tk.now, false);
        const Eigen::Vector3d gt_v = interp_state(Ts, tk.now, true);

        // nearest recorded live belief (publishes lag the tick by the solve time only)
        const RecC* rc = nullptr;
        if (!Cs.empty()) {
            auto it = std::lower_bound(ct.begin(), ct.end(), tk.now);
            size_t j = it - ct.begin();
            size_t best = j < Cs.size() ? j : Cs.size() - 1;
            if (j > 0 && (j >= Cs.size() ||
                          std::abs(ct[j - 1] - tk.now) < std::abs(ct[j] - tk.now)))
                best = j - 1;
            if (std::abs(ct[best] - tk.now) < 0.05) { rc = &Cs[best]; ++n_have_c; }
        }

        const bool rv = tk.q.valid;
        if (tk.r_solved) ++n_solved;
        if (!tk.live.empty() && tk.live.size() >= 16 &&
            tk.rd.n_keyframes == (int)tk.live[3] && tk.rd.n_ego == (int)tk.live[4] &&
            tk.rd.n_peer == (int)tk.live[5])
            ++n_diag_match;

        const Eigen::Matrix3d Pp = rv ? Eigen::Matrix3d(tk.q.cov.block<3, 3>(0, 0))
                                      : Eigen::Matrix3d::Zero();
        const Eigen::Matrix3d Pv = rv ? Eigen::Matrix3d(tk.q.cov.block<3, 3>(3, 3))
                                      : Eigen::Matrix3d::Zero();
        const double r_perr = rv ? (tk.q.p - gt_p).norm() : std::nan("");
        const double r_verr = rv ? (tk.q.v - gt_v).norm() : std::nan("");
        const double np_ = rv ? nees3(tk.q.p - gt_p, Pp) : std::nan("");
        const double nv_ = rv ? nees3(tk.q.v - gt_v, Pv) : std::nan("");
        const double rec_perr = rc ? (rc->p - gt_p).norm() : std::nan("");
        const double rec_verr = rc ? (rc->v - gt_v).norm() : std::nan("");
        const double rec_np = rc ? nees3(rc->p - gt_p, sym3(rc->c)) : std::nan("");
        const double dev = (rv && rc) ? (tk.q.p - rc->p).norm() : std::nan("");

        o << tk.now << ","
          << (tk.r_solved ? 1 : 0) << "," << tk.rd.iterations << "," << tk.rd.n_keyframes << ","
          << tk.rd.n_ego << "," << tk.rd.n_peer << "," << tk.rd.error_before << ","
          << tk.rd.error_after << "," << tk.rd.max_factor_error << ","
          << tk.rd.seed.x() << "," << tk.rd.seed.y() << "," << tk.rd.seed.z() << "," << tk.r_buf;
        for (int i = 0; i < 16; ++i)
            o << "," << (i < (int)tk.live.size() ? tk.live[i] : std::nan(""));
        auto v3 = [&o](const Eigen::Vector3d& v) { o << "," << v.x() << "," << v.y() << "," << v.z(); };
        if (rv) { v3(tk.q.p); v3(tk.q.v); } else { for (int i = 0; i < 6; ++i) o << ",nan"; }
        if (rv) {
            o << "," << Pp(0,0) << "," << Pp(0,1) << "," << Pp(0,2)
              << "," << Pp(1,1) << "," << Pp(1,2) << "," << Pp(2,2)
              << "," << Pv(0,0) << "," << Pv(0,1) << "," << Pv(0,2)
              << "," << Pv(1,1) << "," << Pv(1,2) << "," << Pv(2,2);
        } else { for (int i = 0; i < 12; ++i) o << ",nan"; }
        if (rc) {
            v3(rc->p); v3(rc->v);
            for (int i = 0; i < 6; ++i) o << "," << rc->c[i];
        } else { for (int i = 0; i < 12; ++i) o << ",nan"; }
        v3(gt_p); v3(gt_v);
        o << "," << r_perr << "," << r_verr << "," << rec_perr << "," << rec_verr << ","
          << dev << "," << np_ << "," << nv_ << "," << rec_np;
        // S5 gated published stream: 0 = silent, 1 = accepted solve, 2 = held fallback.
        const int pv = tk.go.publish ? (tk.go.fallback ? 2 : 1) : 0;
        o << "," << pv << "," << tk.go.reason;
        if (tk.go.publish) {
            const double pperr = (tk.go.p - gt_p).norm();
            const double pverr = (tk.go.v - gt_v).norm();
            const double pnp = nees3(tk.go.p - gt_p,
                                     Eigen::Matrix3d(tk.go.cov.block<3, 3>(0, 0)));
            v3(tk.go.p);
            v3(tk.go.v);   // RAL ticket 028: published velocity (e_ZEM on fallback ticks)
            o << "," << pperr << "," << pverr << "," << pnp << "\n";
        } else {
            o << ",nan,nan,nan,nan,nan,nan,nan,nan,nan\n";
        }
    }

    if (fl)
        std::cerr << "fl: err_resets=" << fl->numResets() << " soft_resets=" << fl->numSoftResets()
                  << " dropped_old=" << fl->numDroppedOld() << "\n";
    std::cerr << "replay v2: ticks=" << ticks.size() << " solved=" << n_solved
              << " | diag n_kf/n_ego/n_peer exact-match=" << n_diag_match << "/" << ticks.size()
              << " | ticks with recorded belief=" << n_have_c
              << " | det drops: gate=" << drop_gate << " interp=" << drop_interp << "\n";
    std::cerr << "params: pixel_sigma=" << pixel_sigma << " bearing_deg=" << bearing_deg
              << " sigma_psi_deg=" << sigma_psi_deg << " use_robust=" << use_robust
              << " window_s=" << window_s << " q_c=" << q_c << " order=" << order
              << " | t028: gate_min_peer=" << gate_min_peer << " init_range_m=" << init_range_m
              << " tick_mode=" << tick_mode << " ray_policy=" << ray_policy
              << " ray_param=" << ray_param << "\n";
    return 0;
}
