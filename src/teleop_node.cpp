// MQTT_teleoperation_rl.py の C++/ROS 2 版。
//
//   MQTT(目標ジョイント角, 16-20ms周期) --ZOH--> oversample(output_rate_hz)
//        --> 2次ベッセルLPF --> 速度/関節リミット --> /control/move_js
//   /feedback/joint_states --> 実ジョイント角(FSM判定, MQTTへの state publish)
//
// 1 プロセス 1 ノード。制御ループは専用スレッド(clock_nanosleep 絶対時刻)で回し、
// ROS executor / Paho のコールバックスレッドとは mutex 保護の小さな構造体だけで受け渡す。
#include <pthread.h>
#include <sched.h>
#include <time.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "nlohmann/json.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "mqtt_teleop/MQTT_Client.hpp"
#include "mqtt_teleop/edge_state.hpp"
#include "mqtt_teleop/lowpass_filter.hpp"
#include "mqtt_teleop/msg/metawork_control.hpp"

using json = nlohmann::json;
using namespace std::chrono_literals;

namespace mqtt_teleop
{
namespace
{
std::atomic<bool> g_signal{false};
extern "C" void on_signal(int) {g_signal.store(true);}

using Clock = std::chrono::steady_clock;
using TP = Clock::time_point;

double age_s(TP stamp, TP now) {return std::chrono::duration<double>(now - stamp).count();}

std::string env_or(const char * name, const std::string & def)
{
  const char * v = std::getenv(name);
  return (v && *v) ? std::string(v) : def;
}

int64_t epoch_ms()
{
  return std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}

constexpr size_t kNJ = 6;  // 関節数 (Piper)
constexpr size_t kG = 6;   // filters_ / 配列中の gripper index
using Vec7 = std::array<double, kNJ + 1>;
}  // namespace

struct Params
{
  std::string arm;
  bool liu{false};
  std::string mode{"local"};
  std::string broker_uri, robot_type, robot_uuid, ca_certs_path;
  std::string feedback_topic, control_topic;
  std::string mqtt_recv_topic{"mqtt_recv/joint_states"};
  std::vector<std::string> joint_names;
  std::string gripper_name;

  double output_rate_hz, filter_cutoff_hz, reference_atten_hz;
  bool use_gripper;
  double gripper_scale, gripper_offset, gripper_max_width, gripper_effort;

  double equal_threshold, equal_hold_s;
  double stop_threshold, stop_threshold_hold_s;
  double mqtt_timeout_s, feedback_timeout_s;
  double stop_vel_threshold, stop_converge_eps, stop_hold_s, stop_timeout_s;
  double max_joint_speed;
  std::vector<double> joint_lower, joint_upper;
  double state_pub_rate_hz;
  int64_t rt_priority;
};

class TeleopNode : public rclcpp::Node
{
public:
  TeleopNode() : rclcpp::Node("mqtt_teleop")
  {
    load_params();

    // ---- フィルタ ----
    for (auto & f : filters_) {f.design(p_.filter_cutoff_hz, p_.output_rate_hz);}
    RCLCPP_INFO(
      get_logger(),
      "Bessel2 LPF: fc=%.1f Hz @ fs=%.0f Hz, DC group delay=%.1f ms, gain@%.0fHz=%.1f dB",
      filters_[0].cutoff_hz(), p_.output_rate_hz, filters_[0].group_delay_dc_s() * 1e3,
      p_.reference_atten_hz, filters_[0].gain_db(p_.reference_atten_hz));

    // ---- MQTT ----
    MqttArgs a;
    a.uri = p_.broker_uri;
    a.robot_type = p_.robot_type;
    a.robot_uuid = p_.robot_uuid;
    a.ca_certs_path = p_.ca_certs_path;
    mqtt_ = std::make_unique<MQTT_Client>(p_.arm + "/", p_.mode, a);

    // ---- ROS ----
    cmd_pub_ = create_publisher<sensor_msgs::msg::JointState>(p_.control_topic, rclcpp::QoS(1).reliable());
    mqtt_recv_pub_ = create_publisher<mqtt_teleop::msg::MetaworkControl>(p_.mqtt_recv_topic, rclcpp::QoS(1).reliable());
    fb_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      p_.feedback_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::SharedPtr m) {on_feedback(*m);});

    cmd_arm_.name = p_.joint_names;
    cmd_arm_.position.assign(kNJ, 0.0);
    cmd_arm_.effort.assign(kNJ, 0.0);
    cmd_full_ = cmd_arm_;
    cmd_full_.name.push_back(p_.gripper_name);
    cmd_full_.position.push_back(0.0);
    cmd_full_.effort.push_back(p_.gripper_effort);

    if (p_.state_pub_rate_hz > 0.0) {
      state_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(1.0 / p_.state_pub_rate_hz)),
        [this]() {publish_initialize();});
    }
  }

  ~TeleopNode() override
  {
    finished_ = true;
    if (control_thread_.joinable()) {control_thread_.join();}
  }

  void start()
  {
    mqtt_->start_mqtt();
    control_thread_ = std::thread(&TeleopNode::control_loop, this);
  }

  void request_exit() {exit_requested_ = true;}
  bool finished() const {return finished_;}
  double exit_wait_s() const {return p_.stop_timeout_s + 2.0;}

  // main から: 終了処理 (KeyboardInterrupt 節に相当)
  void shutdown_mqtt(bool error)
  {
    const json m = {{"state", error ? "error" : "stop"}};
    mqtt_->publish_message(m);
    mqtt_->publish_message(m);
    mqtt_->unregister_message();
    mqtt_->stop_mqtt();
  }

private:
  // ================================================================ params
  void load_params()
  {
    auto S = [this](const char * n, const std::string & d) -> std::string {return declare_parameter<std::string>(n, d);};
    auto D = [this](const char * n, double d) {return declare_parameter<double>(n, d);};
    auto B = [this](const char * n, bool d) {return declare_parameter<bool>(n, d);};

    p_.arm = S("arm", "");
    if (p_.arm != "right" && p_.arm != "left") {
      throw std::runtime_error("Please specify parameter arm:=right or arm:=left");
    }
    p_.liu = B("liu", false);
    p_.mode = S("mode", "local");
    p_.broker_uri = S("broker_uri", env_or("MQTT_URI", "wss://localhost:8333"));
    RCLCPP_INFO(get_logger(), "BROKER_URI: %s",p_.broker_uri.c_str());
    const bool right = p_.arm == "right";
    p_.robot_type = S("robot_type", "");
    if (p_.robot_type.empty()) {p_.robot_type = env_or("ROBOT_TYPE", right ? "piper_right" : "piper_left");}
    p_.robot_uuid = S("robot_uuid", "");
    if (p_.robot_uuid.empty()) {
      p_.robot_uuid = env_or("ROBOT_UUID", right ? "MA100101000019005100402" : "MA100101000019005100010");
    }
    p_.ca_certs_path = S("ca_certs_path", "");
    if (p_.ca_certs_path.empty()) {p_.ca_certs_path = env_or("CA_CERTS_PATH", "");}

    // 相対名: namespace で右腕/左腕の agx_arm_ctrl を切り替える
    p_.feedback_topic = S("feedback_topic", "feedback/joint_states");
    p_.control_topic = S("control_topic", "control/move_js");
    p_.joint_names = declare_parameter<std::vector<std::string>>(
      "joint_names", std::vector<std::string>{"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"});
    if (p_.joint_names.size() != kNJ) {throw std::runtime_error("joint_names must have 6 entries");}
    p_.gripper_name = S("gripper_name", "gripper");

    p_.output_rate_hz = D("output_rate_hz", 200.0);
    p_.filter_cutoff_hz = D("filter_cutoff_hz", 15.0);
    p_.reference_atten_hz = D("reference_atten_hz", 60.0);

    p_.use_gripper = B("use_gripper", true);
    // 旧: finger_pos[mm] = tool*0.85 + 0.4 -> agx_arm_ctrl は幅[m] (0..0.1)
    p_.gripper_scale = D("gripper_scale", 0.85e-3);
    p_.gripper_offset = D("gripper_offset", 0.4e-3);
    p_.gripper_max_width = D("gripper_max_width", 0.1);
    p_.gripper_effort = D("gripper_effort", 1.0);

    p_.equal_threshold = D("equal_threshold", 0.01);       // acquired -> running
    p_.equal_hold_s = D("equal_hold_s", 0.1);
    p_.stop_threshold = D("stop_threshold", 0.10);         // running -> stopping
    p_.stop_threshold_hold_s = D("stop_threshold_hold_s", 0.05);
    p_.mqtt_timeout_s = D("mqtt_timeout_s", 0.2);          // 0 で無効
    p_.feedback_timeout_s = D("feedback_timeout_s", 0.2);
    p_.stop_vel_threshold = D("stop_vel_threshold", 0.05);  // stopping -> acquired
    p_.stop_converge_eps = D("stop_converge_eps", 0.001);
    p_.stop_hold_s = D("stop_hold_s", 0.2);
    p_.stop_timeout_s = D("stop_timeout_s", 5.0);
    p_.max_joint_speed = D("max_joint_speed", 3.0);        // rad/s, 0 で無効
    p_.joint_lower = declare_parameter<std::vector<double>>("joint_lower", std::vector<double>{});
    p_.joint_upper = declare_parameter<std::vector<double>>("joint_upper", std::vector<double>{});
    if (p_.joint_lower.size() != p_.joint_upper.size() ||
      (!p_.joint_lower.empty() && p_.joint_lower.size() != kNJ))
    {
      throw std::runtime_error("joint_lower/joint_upper must both be empty or have 6 entries");
    }
    p_.state_pub_rate_hz = D("state_pub_rate_hz", 10.0);
    p_.rt_priority = declare_parameter<int64_t>("rt_priority", 0);

    // MQTT payload のキー (Python 版 msg_key_* と同じ規則)
    if (right) {
      key_state_ = "state"; key_model_ = "model";
      key_jf_ = p_.liu ? "joint_feedback" : "joints";
    } else {
      key_state_ = p_.liu ? "state_left" : "state";
      key_model_ = p_.liu ? "model_left" : "model";
      key_jf_ = p_.liu ? "joint_feedback_left" : "joints";
    }
    ext_joint_ = p_.arm + "/joint";
    ext_tool_ = p_.arm + "/tool";
  }

  // ============================================================== feedback
  struct Feedback
  {
    bool valid{false};
    std::array<double, kNJ> pos{};
    std::array<double, kNJ> vel{};
    double gripper{0.0};
    bool has_gripper{false};
    TP stamp{};
  };

  void on_feedback(const sensor_msgs::msg::JointState & m)
  {
    RCLCPP_DEBUG(get_logger(), "feedback: %zu joints, %zu vel", m.name.size(), m.velocity.size());
    Feedback f;
    std::array<bool, kNJ> got{};
    constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
    f.vel.fill(kNaN);
    for (size_t k = 0; k < m.name.size() && k < m.position.size(); ++k) {
      for (size_t i = 0; i < kNJ; ++i) {
        if (m.name[k] == p_.joint_names[i]) {
          f.pos[i] = m.position[k];
          if (k < m.velocity.size()) {f.vel[i] = m.velocity[k];}
          got[i] = true;
        }
      }
      if (m.name[k] == p_.gripper_name) {
        f.gripper = m.position[k];
        f.has_gripper = true;
      }
    }
    f.valid = std::all_of(got.begin(), got.end(), [](bool b) {return b;});
    f.stamp = Clock::now();
    if (!f.valid) {return;}

    std::lock_guard<std::mutex> g(fb_mtx_);
    // velocity が無い/NaN の場合は位置差分で代用
    if (fb_.valid) {
      const double dt = age_s(fb_.stamp, f.stamp);
      for (size_t i = 0; i < kNJ; ++i) {
        if (std::isnan(f.vel[i])) {f.vel[i] = dt > 1e-4 ? (f.pos[i] - fb_.pos[i]) / dt : 0.0;}
      }
    } else {
      for (auto & v : f.vel) {if (std::isnan(v)) {v = 0.0;}}
    }
    fb_ = f;
  }

  Feedback get_feedback()
  {
    std::lock_guard<std::mutex> g(fb_mtx_);
    return fb_;
  }

  // ======================================================== control thread
  static void add_ns(timespec & t, int64_t ns)
  {
    t.tv_nsec += ns;
    while (t.tv_nsec >= 1000000000L) {t.tv_nsec -= 1000000000L; t.tv_sec += 1;}
  }

  void setup_realtime()
  {
    if (p_.rt_priority <= 0) {return;}
    sched_param sp{};
    sp.sched_priority = static_cast<int>(p_.rt_priority);
    const int rc = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
    if (rc != 0) {
      RCLCPP_WARN(get_logger(), "SCHED_FIFO(%ld) failed: %s (rtprio limit / CAP_SYS_NICE?)",
        static_cast<long>(p_.rt_priority), std::strerror(rc));
    } else {
      RCLCPP_INFO(get_logger(), "control thread: SCHED_FIFO priority %ld", static_cast<long>(p_.rt_priority));
    }
  }

  void control_loop()
  {
    setup_realtime();
    const int64_t period_ns = static_cast<int64_t>(1e9 / p_.output_rate_hz);
    timespec next{};
    clock_gettime(CLOCK_MONOTONIC, &next);
    int64_t overruns = 0;
    while (!finished_) {
      add_ns(next, period_ns);
      while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr) == EINTR) {}
      timespec now{};
      clock_gettime(CLOCK_MONOTONIC, &now);
      const int64_t late_ns = (now.tv_sec - next.tv_sec) * 1000000000L + (now.tv_nsec - next.tv_nsec);
      if (late_ns > 5 * period_ns) {  // 大幅遅延: 位相を取り直す
        next = now;
        if (++overruns % 100 == 1) {
          RCLCPP_WARN(get_logger(), "control loop overrun %.1f ms (count=%ld)", late_ns * 1e-6, static_cast<long>(overruns));
        }
      }
      tick();
    }
  }

  void tick()
  {
    const TP chrono_now = Clock::now();
    const EdgeState st = mqtt_->get_state();
    const auto ros_now = now();
    switch (st) {
      case EdgeState::ACQUIRED: tick_acquired(chrono_now); break;
      case EdgeState::RUNNING: tick_running(chrono_now); break;
      case EdgeState::STOPPING: tick_stopping(chrono_now); break;
      case EdgeState::EXITING: finished_ = true; break;
      default: break;  // まだ acquire されていない
    }
    // get_pose()のTeleopPose構造体のjoint.monotonicを見てmonotonic_stamp_prev_より
    // 進んでいたら、新しくメッセージを受信したとしてデバッグ用にそのメッセージを
    // ros2でpublishする
    const TeleopPose tgt = mqtt_->get_pose();
    if (tgt.joint.valid && tgt.joint.stamp.monotonic > monotonic_stamp_prev_) {
      monotonic_stamp_prev_ = tgt.joint.stamp.monotonic;
      mqtt_teleop::msg::MetaworkControl msg{};
      msg.header.stamp.sec = static_cast<int32_t>(tgt.joint.stamp.mqtt_sec.count());
      msg.header.stamp.nanosec = static_cast<uint32_t>(tgt.joint.stamp.mqtt_nsec.count());
      msg.data.assign(tgt.joint.data.begin(), tgt.joint.data.end());
      mqtt_recv_pub_->publish(msg);
    }
    if (exit_requested_) {
      // RUNNING/STOPPING 中は soft stop 完了後に tick_stopping() が EXITING にする
      const EdgeState s = mqtt_->get_state();
      if (s != EdgeState::RUNNING && s != EdgeState::STOPPING && s != EdgeState::EXITING) {
        mqtt_->set_state(EdgeState::EXITING);
        finished_ = true;
      }
    }
  }

  // ---------------------------------------------------------------- helpers
  Vec7 to_ros_domain(const TeleopPose & t) const
  {
    Vec7 v{};
    for (size_t i = 0; i < kNJ; ++i) {v[i] = t.joint.data[i];}
    v[kG] = t.tool.data[0] * p_.gripper_scale + p_.gripper_offset;  // m
    return v;
  }

  static bool finite_pose(const TeleopPose & t, bool use_gripper)
  {
    for (double x : t.joint.data) {if (!std::isfinite(x)) {return false;}}
    return !(use_gripper && t.tool.valid && !std::isfinite(t.tool.data[0]));
  }

  Vec7 step(const Vec7 & in, bool use_gripper)
  {
    const double dt = 1.0 / p_.output_rate_hz;
    Vec7 y = last_out_;
    for (size_t i = 0; i < kNJ; ++i) {
      double v = filters_[i].process(in[i]);
      if (p_.max_joint_speed > 0.0) {
        const double d = p_.max_joint_speed * dt;
        v = std::clamp(v, last_out_[i] - d, last_out_[i] + d);
      }
      if (!p_.joint_lower.empty()) {v = std::clamp(v, p_.joint_lower[i], p_.joint_upper[i]);}
      y[i] = v;
    }
    if (use_gripper) {
      y[kG] = std::clamp(filters_[kG].process(in[kG]), 0.0, p_.gripper_max_width);
    }
    last_out_ = y;
    return y;
  }

  void publish_cmd(const Vec7 & y, bool use_gripper)
  {
    auto & msg = use_gripper ? cmd_full_ : cmd_arm_;
    msg.header.stamp = now();
    for (size_t i = 0; i < kNJ; ++i) {msg.position[i] = y[i];}
    if (use_gripper) {msg.position[kG] = y[kG];}
    cmd_pub_->publish(msg);
  }

  // ---------------------------------------------------------------- ACQUIRED
  // ros2 へは何も出力しない。MQTT 目標が実ジョイントにほぼ一致したら RUNNING。
  void tick_acquired(TP now)
  {
    if (mqtt_->consume_acquire_request()) {equal_since_.reset();}
    const Feedback fb = get_feedback();
    const TeleopPose tgt = mqtt_->get_pose();

    bool ok = fb.valid && age_s(fb.stamp, now) < p_.feedback_timeout_s && tgt.joint.valid &&
      finite_pose(tgt, false);
    if (ok && p_.mqtt_timeout_s > 0.0) {ok = age_s(tgt.joint.stamp.monotonic, now) < p_.mqtt_timeout_s;}
    bool equal = ok;
    for (size_t i = 0; equal && i < kNJ; ++i) {
      equal = std::fabs(tgt.joint.data[i] - fb.pos[i]) < p_.equal_threshold;
    }
    if (!equal) {equal_since_.reset(); return;}
    if (!equal_since_) {equal_since_ = now;}
    if (age_s(*equal_since_, now) >= p_.equal_hold_s) {start_running(fb, tgt);}
  }

  void start_running(const Feedback & fb, const TeleopPose & tgt)
  {
    const Vec7 in = to_ros_domain(tgt);
    for (size_t i = 0; i < kNJ; ++i) {
      filters_[i].reset(fb.pos[i]);  // 現在の実関節角で初期化 -> 起動時ステップなし
      last_out_[i] = fb.pos[i];
    }
    gripper_active_ = false;
    last_input_ = in;
    over_since_.reset();
    equal_since_.reset();
    mqtt_->set_state(EdgeState::RUNNING);
    RCLCPP_INFO(get_logger(), "##### Robot Ready. state: RUNNING");

    const int64_t t = epoch_ms();
    mqtt_->publish_message({{"time", t}, {key_state_, "ready"}, {key_model_, "agilex_piper"}, {"ext", ext_joint_}});
    mqtt_->publish_message({{"time", t}, {key_state_, "ready"}, {key_model_, "agilex_piper"}, {"ext", ext_tool_}});
  }

  // ----------------------------------------------------------------- RUNNING
  void tick_running(TP now)
  {
    const Feedback fb = get_feedback();
    const TeleopPose tgt = mqtt_->get_pose();

    std::string reason;
    if (exit_requested_) {
      reason = "exit requested";
    } else if (mqtt_->consume_acquire_request()) {
      reason = "re-acquired (dev/<robot-id> received)";
    } else if (!tgt.joint.valid || !finite_pose(tgt, p_.use_gripper)) {
      reason = "invalid target (NaN/Inf)";
    } else if (p_.mqtt_timeout_s > 0.0 && age_s(tgt.joint.stamp.monotonic, now) > p_.mqtt_timeout_s) {
      reason = "MQTT target timeout";
    } else if (!fb.valid || age_s(fb.stamp, now) > p_.feedback_timeout_s) {
      reason = "feedback timeout";
    }
    if (!reason.empty()) {
      begin_stopping(reason, now);
      tick_stopping(now);  // 発行を途切れさせない
      return;
    }

    // ZOH: 最新の MQTT 目標をそのまま保持して oversample
    Vec7 in = to_ros_domain(tgt);
    const bool use_gripper = p_.use_gripper && tgt.tool.valid;
    if (use_gripper && !gripper_active_) {
      filters_[kG].reset(fb.has_gripper ? fb.gripper : in[kG]);
      last_out_[kG] = fb.has_gripper ? fb.gripper : in[kG];
      gripper_active_ = true;
    }
    last_input_ = in;
    const Vec7 y = step(in, use_gripper);
    publish_cmd(y, use_gripper);

    // 目標(フィルタ出力) と 実ジョイントの差が大きい状態が続いたら STOPPING
    double max_err = 0.0;
    for (size_t i = 0; i < kNJ; ++i) {max_err = std::max(max_err, std::fabs(y[i] - fb.pos[i]));}
    if (max_err > p_.stop_threshold) {
      if (!over_since_) {over_since_ = now;}
      if (age_s(*over_since_, now) >= p_.stop_threshold_hold_s) {
        begin_stopping("tracking error " + std::to_string(max_err) + " rad", now);
      }
    } else {
      over_since_.reset();
    }
  }

  // ---------------------------------------------------------------- STOPPING
  // 目標の publish は止めず、入力を直前の値に固定してフィルタ遅れを収束させる。
  void begin_stopping(const std::string & reason, TP now)
  {
    RCLCPP_WARN(get_logger(), "RUNNING -> STOPPING: %s", reason.c_str());
    held_input_ = last_input_;
    stopping_since_ = now;
    stop_ok_since_.reset();
    over_since_.reset();
    mqtt_->set_state(EdgeState::STOPPING);
  }

  void tick_stopping(TP now)
  {
    const Feedback fb = get_feedback();
    const bool use_gripper = p_.use_gripper && gripper_active_;
    const Vec7 y = step(held_input_, use_gripper);
    publish_cmd(y, use_gripper);

    bool converged = true;
    for (size_t i = 0; i < kNJ; ++i) {
      converged = converged && std::fabs(y[i] - held_input_[i]) < p_.stop_converge_eps;
    }
    bool still = fb.valid;
    for (size_t i = 0; still && i < kNJ; ++i) {still = std::fabs(fb.vel[i]) < p_.stop_vel_threshold;}

    bool done = false;
    if (converged && still) {
      if (!stop_ok_since_) {stop_ok_since_ = now;}
      done = age_s(*stop_ok_since_, now) >= p_.stop_hold_s;
    } else {
      stop_ok_since_.reset();
    }
    if (!done && age_s(stopping_since_, now) > p_.stop_timeout_s) {
      RCLCPP_WARN(get_logger(), "STOPPING timeout (%.1fs): robot velocity did not settle", p_.stop_timeout_s);
      done = true;
    }
    if (!done) {return;}

    mqtt_->consume_acquire_request();
    equal_since_.reset();
    if (exit_requested_) {
      mqtt_->set_state(EdgeState::EXITING);
      finished_ = true;
      RCLCPP_INFO(get_logger(), "STOPPING -> EXITING");
    } else {
      mqtt_->set_state(EdgeState::ACQUIRED);
      RCLCPP_INFO(get_logger(), "STOPPING -> ACQUIRED");
    }
  }

  // ======================================================== MQTT state pub
  // ACQUIRED 中: 実ジョイント値を frontend へ送る(旧 "initialize" メッセージ)
  void publish_initialize()
  {
    if (mqtt_->get_state() != EdgeState::ACQUIRED) {return;}
    const Feedback fb = get_feedback();
    if (!fb.valid) {return;}
    const json jf = json::array({fb.pos[0], fb.pos[1], fb.pos[2], fb.pos[3], fb.pos[4], fb.pos[5]});
    const int64_t t = epoch_ms();
    mqtt_->publish_message({{"time", t}, {key_state_, "initialize"}, {key_model_, "agilex_piper"}, {key_jf_, jf}, {"ext", ext_joint_}});
    mqtt_->publish_message({{"time", t}, {key_state_, "initialize"}, {key_model_, "agilex_piper"}, {key_jf_, jf}, {"ext", ext_tool_}});
  }

  // ================================================================ members
  Params p_;
  std::string key_state_, key_model_, key_jf_, ext_joint_, ext_tool_;

  std::unique_ptr<MQTT_Client> mqtt_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr cmd_pub_;
  rclcpp::Publisher<mqtt_teleop::msg::MetaworkControl>::SharedPtr mqtt_recv_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr fb_sub_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  sensor_msgs::msg::JointState cmd_arm_, cmd_full_;

  std::mutex fb_mtx_;
  Feedback fb_;

  // 以下は control thread のみが触る
  std::array<Bessel2LowPass, kNJ + 1> filters_;
  Vec7 last_out_{}, last_input_{}, held_input_{};
  bool gripper_active_{false};
  std::optional<TP> equal_since_, over_since_, stop_ok_since_;
  TP stopping_since_{};

  std::thread control_thread_;
  std::atomic<bool> exit_requested_{false};
  std::atomic<bool> finished_{false};
  TeleopStamp::Clock::time_point monotonic_stamp_prev_{TeleopStamp::Clock::time_point::min()};
};

}  // namespace mqtt_teleop

int main(int argc, char ** argv)
{
  // SIGINT は自前で扱い、soft stop -> unregister してから ROS を終了する
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, mqtt_teleop::on_signal);
  std::signal(SIGTERM, mqtt_teleop::on_signal);

  std::shared_ptr<mqtt_teleop::TeleopNode> node;
  try {
    node = std::make_shared<mqtt_teleop::TeleopNode>();
  } catch (const std::exception & e) {
    std::cerr << "Init error: " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  std::thread spin_thread([&exec]() {exec.spin();});

  int rc = 0;
  try {
    node->start();
    while (!mqtt_teleop::g_signal.load() && rclcpp::ok() && !node->finished()) {
      std::this_thread::sleep_for(50ms);
    }
    if (mqtt_teleop::g_signal.load()) {std::cout << "SIGINT Received. Stopping" << std::endl;}
    node->request_exit();
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(node->exit_wait_s());
    while (!node->finished() && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(20ms);
    }
    node->shutdown_mqtt(false);
  } catch (const std::exception & e) {
    std::cerr << "MQTT Recv Error: " << e.what() << std::endl;
    node->shutdown_mqtt(true);
    rc = 1;
  }

  exec.cancel();
  spin_thread.join();
  node.reset();
  rclcpp::shutdown();
  return rc;
}
