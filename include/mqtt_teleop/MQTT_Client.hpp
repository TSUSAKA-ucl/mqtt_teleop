#pragma once
// MQTT_Client.py の C++ 版。メソッド名は Python 版に揃えてある。
//
//   Python                          C++
//   ------------------------------  -----------------------------------------
//   __init__(arm, mode, args)       MQTT_Client(arm, mode, args)
//   start_mqtt()                    start_mqtt()
//   subscribe_all_on_list()         subscribe_all_on_list()
//   unsubscribe_all_on_list()       unsubscribe_all_on_list()
//   on_connect()                    on_connect()      (Paho の connected() から)
//   on_disconnect()                 on_disconnect()   (Paho の connection_lost() から)
//   on_acquire_topic()              on_acquire_topic()
//   on_message()                    on_message()
//   publish_message()               publish_message()
//   unregister_message()            unregister_message()
//   set_time_offset()               set_time_offset()
//   self.lock / self.state          get_state()/set_state() (内部 mutex)
//   self.pose (numpy 16要素)        get_pose() -> TeleopPose (コピーを返す)
#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <mqtt/async_client.h>
#include <nlohmann/json.hpp>

#include "mqtt_teleop/edge_state.hpp"

namespace mqtt_teleop {

struct MqttArgs
{
  // tcp:// | ssl:// | ws:// | wss://  (Python 版は websockets+TLS なので既定は wss://)
  std::string uri{"wss://localhost:8333"};
  std::string robot_type{"piper_right"};
  std::string robot_uuid{"MA100101000019005100402"};
  std::string ca_certs_path{};  // 空なら /etc/ssl/certs のみ
};

// Python 版 self.pose[8:14] / self.pose[15] に相当。「6関節 + ツール1」に整理。
struct TeleopPose
{
  using Clock = std::chrono::steady_clock;
  std::array<double, 6> joint{};
  double tool{0.0};
  bool joint_valid{false};
  bool tool_valid{false};
  Clock::time_point joint_stamp{};  // 最後に joint を受信した時刻
  Clock::time_point tool_stamp{};
};

class MQTT_Client : public mqtt::callback
{
public:
  static constexpr const char * MQTT_DEVICE_TOPIC_HDR = "dev";
  static constexpr const char * MGR_REGISTER_TOPIC = "mgr/register";
  static constexpr const char * MGR_UNREGISTER_TOPIC = "mgr/unregister";

  // arm は "right/" または "left/"
  MQTT_Client(const std::string & arm, const std::string & mode, const MqttArgs & args);
  ~MQTT_Client() override;

  void start_mqtt();
  void stop_mqtt();

  void subscribe_all_on_list();
  void unsubscribe_all_on_list();

  void on_connect();
  void on_disconnect(const std::string & cause);
  void on_acquire_topic(const std::string & payload);
  void on_message(const std::string & topic, const std::string & payload);

  void publish_message(const nlohmann::json & payload);
  void unregister_message();
  void set_time_offset(double time_offset) { time_vr_robot_offset_ = time_offset; }

  // --- state / pose (thread-safe) -------------------------------------
  EdgeState get_state() const;
  void set_state(EdgeState s);
  TeleopPose get_pose() const;
  std::string user_uuid() const;

  // RUNNING/STOPPING 中に acquire(dev/<robot-id>)を受けたら立つ。読むと同時にクリア。
  // Python 版は無条件で state=ACQUIRED にしていたが、状態遷移は main の FSM が行う。
  bool consume_acquire_request();

  const std::string ROBOT_TYPE;
  const std::string ROBOT_UUID;
  const std::string DEV_ACQUIRE_TOPIC;
  const std::string MQTT_ROBOT_STATE_TOPIC;

private:
  // mqtt::callback
  void connected(const std::string & cause) override;
  void connection_lost(const std::string & cause) override;
  void message_arrived(mqtt::const_message_ptr msg) override;
  void delivery_complete(mqtt::delivery_token_ptr) override {}

  MqttArgs args_;
  std::string joint_topic_;
  std::string tool_topic_;
  std::string mode_;
  std::atomic<double> time_vr_robot_offset_{0.0};

  mutable std::mutex lock_;
  EdgeState state_{EdgeState::INIT};
  TeleopPose pose_;
  std::string user_uuid_;
  std::string ctrl_joint_topic_;  // control/<arm>joint/<USER_UUID>
  std::string ctrl_tool_topic_;
  std::vector<std::string> subscription_list_;
  std::atomic<bool> acquire_request_{false};

  std::unique_ptr<mqtt::async_client> cli_;
};

}  // namespace mqtt_teleop
