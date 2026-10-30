#include "mqtt_teleop/MQTT_Client.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <iostream>
#include <limits>
#include <stdexcept>

using json = nlohmann::json;

namespace mqtt_teleop
{
namespace
{
std::string utc_now_iso8601()
{
  using namespace std::chrono;
  const auto now = system_clock::now();
  const std::time_t t = system_clock::to_time_t(now);
  std::tm tm{};
  gmtime_r(&t, &tm);
  const long long us = duration_cast<microseconds>(now.time_since_epoch()).count() % 1000000;
  char buf[32];
  std::strftime(buf, sizeof buf, "%Y-%m-%dT%H:%M:%S", &tm);
  char out[64];
  std::snprintf(out, sizeof out, "%s.%06lld+00:00", buf, us);
  return out;
}

bool starts_with(const std::string & s, const std::string & p)
{
  return s.size() >= p.size() && s.compare(0, p.size(), p) == 0;
}
}  // namespace




MQTT_Client::MQTT_Client(const std::string & arm, const std::string & mode, const MqttArgs & args)
: ROBOT_TYPE(args.robot_type),
  ROBOT_UUID(args.robot_uuid),
  DEV_ACQUIRE_TOPIC(std::string(MQTT_DEVICE_TOPIC_HDR) + "/" + args.robot_uuid),
  MQTT_ROBOT_STATE_TOPIC("robot/" + args.robot_uuid),
  args_(args),
  joint_topic_(arm + "joint/"),
  tool_topic_(arm + "tool/"),
  mode_(mode)
{
  cli_ = std::make_unique<mqtt::async_client>(args_.uri, args_.robot_uuid + "-" + arm);
  cli_->set_callback(*this);
}

MQTT_Client::~MQTT_Client() { stop_mqtt(); }

// ---------------------------------------------------------------- state --
EdgeState MQTT_Client::get_state() const
{
  std::lock_guard<std::mutex> g(lock_);
  return state_;
}

void MQTT_Client::set_state(EdgeState s)
{
  std::lock_guard<std::mutex> g(lock_);
  state_ = s;
}

TeleopPose MQTT_Client::get_pose() const
{
  std::lock_guard<std::mutex> g(lock_);
  return pose_;
}

std::string MQTT_Client::user_uuid() const
{
  std::lock_guard<std::mutex> g(lock_);
  return user_uuid_;
}

bool MQTT_Client::consume_acquire_request() { return acquire_request_.exchange(false); }

// ----------------------------------------------------------- connection --
void MQTT_Client::start_mqtt()
{
  set_state(EdgeState::STARTING);
  std::cout << "MQTT Client starting...\nMQTT Client connecting to: " << args_.uri << std::endl;

  auto builder = mqtt::connect_options_builder()
                   .keep_alive_interval(std::chrono::seconds(60))
                   .clean_session(true)
                   .automatic_reconnect(std::chrono::seconds(1), std::chrono::seconds(30));

  if (starts_with(args_.uri, "ssl://") || starts_with(args_.uri, "wss://")) {
    mqtt::ssl_options_builder sb;
    sb.enable_server_cert_auth(true);
    sb.verify(true);
    if (!args_.ca_certs_path.empty()) {
      sb.trust_store(args_.ca_certs_path);
    }
    sb.ca_path("/etc/ssl/certs");
    builder.ssl(sb.finalize());
  }

  auto tok = cli_->connect(builder.finalize());
  if (!tok->wait_for(std::chrono::seconds(10))) {
    throw std::runtime_error("MQTT connect timeout: " + args_.uri);
  }
  // 以降の登録処理は Paho の connected() コールバック -> on_connect() で行う
}

void MQTT_Client::stop_mqtt()
{
  try {
    if (cli_ && cli_->is_connected()) {
      cli_->disconnect()->wait_for(std::chrono::seconds(2));
    }
  } catch (const std::exception & e) {
    std::cerr << "MQTT disconnect: " << e.what() << std::endl;
  }
}

void MQTT_Client::connected(const std::string &) { on_connect(); }
void MQTT_Client::connection_lost(const std::string & cause) { on_disconnect(cause); }

void MQTT_Client::message_arrived(mqtt::const_message_ptr msg)
{
  const std::string & topic = msg->get_topic();
  if (topic == DEV_ACQUIRE_TOPIC) {
    on_acquire_topic(msg->to_string());
  } else {
    on_message(topic, msg->to_string());
  }
}

void MQTT_Client::subscribe_all_on_list()
{
  std::vector<std::string> list;
  {
    std::lock_guard<std::mutex> g(lock_);
    list = subscription_list_;
  }
  for (const auto & t : list) {
    cli_->subscribe(t, 0);  // コールバックスレッド内から呼ぶので wait しない
    std::cout << "Subscribed to topic: " << t << std::endl;
  }
}

void MQTT_Client::unsubscribe_all_on_list()
{
  std::vector<std::string> list;
  {
    std::lock_guard<std::mutex> g(lock_);
    list = subscription_list_;
  }
  for (const auto & t : list) {
    cli_->unsubscribe(t);
    std::cout << "Unsubscribed from topic: " << t << std::endl;
  }
}

void MQTT_Client::on_connect()
{
  std::cout << "###### enter on_connect" << std::endl;
  {
    // 再接続(自動)で ACQUIRED/RUNNING を巻き戻さない
    std::lock_guard<std::mutex> g(lock_);
    if (state_ == EdgeState::INIT || state_ == EdgeState::STARTING) {
      state_ = EdgeState::REGISTRATION;
    }
  }
  try {
    const json my_info = {
      {"date", utc_now_iso8601()}, {"devType", "robot"}, {"type", ROBOT_TYPE},
      {"version", "0.1.1"},        {"devId", ROBOT_UUID}};
    cli_->publish(MGR_REGISTER_TOPIC, my_info.dump(), 1, false);
    std::cout << "Publish robot registeration: " << my_info.dump() << std::endl;
    cli_->subscribe(DEV_ACQUIRE_TOPIC, 0);
    subscribe_all_on_list();
  } catch (const std::exception & e) {
    std::cerr << "on_connect failed: " << e.what() << std::endl;
  }
  // 現状 registration 完了通知は無いので、ここで REGISTERED にする(Python 版と同じ)
  std::lock_guard<std::mutex> g(lock_);
  if (state_ == EdgeState::REGISTRATION) {
    state_ = EdgeState::REGISTERED;
  }
}

void MQTT_Client::on_disconnect(const std::string & cause)
{
  std::cerr << "Unexpected disconnection. " << cause << std::endl;
}

// -------------------------------------------------------------- acquire --
void MQTT_Client::on_acquire_topic(const std::string & payload)
{
  std::string from_dev_id;
  try {
    const json js = json::parse(payload);
    if (!js.is_object()) {return;}
    from_dev_id = js.value("devId", std::string());
    const std::string controller = js.value("controller", std::string());
    if (from_dev_id.empty() || controller != "browser") {return;}
  } catch (const json::exception & e) {
    std::cerr << "XXX Subscribe " << DEV_ACQUIRE_TOPIC << " failed: " << e.what() << std::endl;
    return;
  }

  std::cout << "Received message from " << from_dev_id << " on topic " << DEV_ACQUIRE_TOPIC
            << std::endl;

  bool new_user;
  {
    std::lock_guard<std::mutex> g(lock_);
    new_user = (from_dev_id != user_uuid_);
  }
  if (new_user) {
    std::cout << "## Capture New Control Request: " << from_dev_id << std::endl;
    unsubscribe_all_on_list();
    {
      std::lock_guard<std::mutex> g(lock_);
      user_uuid_ = from_dev_id;
      ctrl_joint_topic_ = "control/" + joint_topic_ + user_uuid_;
      ctrl_tool_topic_ = "control/" + tool_topic_ + user_uuid_;
      subscription_list_ = {ctrl_joint_topic_, ctrl_tool_topic_};
      pose_ = TeleopPose{};  // 前ユーザーの目標値を捨てる
    }
    subscribe_all_on_list();
  }

  std::lock_guard<std::mutex> g(lock_);
  if (state_ == EdgeState::RUNNING || state_ == EdgeState::STOPPING) {
    acquire_request_ = true;  // 遷移は main の FSM が行う
  } else if (state_ != EdgeState::EXITING) {
    state_ = EdgeState::ACQUIRED;
  }
}

// -------------------------------------------------------------- message --
void MQTT_Client::on_message(const std::string & topic, const std::string & payload)
{
  auto recv_now = std::chrono::system_clock::now();
  auto from_epoch = recv_now.time_since_epoch();
  auto SetStamp = [from_epoch](TeleopStamp & stamp, double payload_stamp) {
    stamp.monotonic = TeleopStamp::Clock::now();
    stamp.payload = payload_stamp;
    stamp.mqtt_sec = std::chrono::duration_cast<std::chrono::seconds>(from_epoch);
    stamp.mqtt_nsec =
	std::chrono::duration_cast<std::chrono::nanoseconds>(from_epoch - stamp.mqtt_sec);
  };

  std::string jt, tt;
  {
    std::lock_guard<std::mutex> g(lock_);
    jt = ctrl_joint_topic_;
    tt = ctrl_tool_topic_;
  }
  constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
  try {
    if (!jt.empty() && topic == jt) {
      const json js = json::parse(payload);
      const auto & joints = js.at("joint");
      if (!joints.is_array() || joints.size() < 6) {return;}
      std::array<double, 6> th{};
      for (size_t i = 0; i < 6; ++i) {
        th[i] = joints[i].is_number() ? joints[i].get<double>() : kNaN;  // NaN は FSM が検出
      }
      const auto payload_stamp = js.value("stamp", 0.0);
      std::lock_guard<std::mutex> g(lock_);
      pose_.joint.data = th;
      pose_.joint.valid = true;
      SetStamp(pose_.joint.stamp, payload_stamp);
    } else if (!tt.empty() && topic == tt) {
      const json js = json::parse(payload);
      const auto & v = js.at("tool");
      const double tool = v.is_number() ? v.get<double>() : kNaN;
      const auto payload_stamp = js.value("stamp", 0.0);
      std::lock_guard<std::mutex> g(lock_);
      pose_.tool.data.at(0) = tool;
      pose_.tool.valid = true;
      SetStamp(pose_.tool.stamp, payload_stamp);
    } else {
      std::cout << "not subscribe msg " << topic << std::endl;
    }
  } catch (const json::exception & e) {
    std::cerr << "bad payload on " << topic << ": " << e.what() << std::endl;
  }
}

// -------------------------------------------------------------- publish --
void MQTT_Client::publish_message(const json & payload)
{
  try {
    cli_->publish(MQTT_ROBOT_STATE_TOPIC, payload.dump(), 0, false);
  } catch (const std::exception & e) {
    std::cerr << "publish_message failed: " << e.what() << std::endl;
  }
}

void MQTT_Client::unregister_message()
{
  const json p = {{"date", utc_now_iso8601()}, {"devType", "robot"}, {"type", ROBOT_TYPE},
                  {"version", "0.1.1"},        {"devId", ROBOT_UUID},  {"status", "unregister"}};
  try {
    auto tok = cli_->publish(MGR_UNREGISTER_TOPIC, p.dump(), 1, false);
    tok->wait_for(std::chrono::seconds(2));
    std::cout << "Publish robot unregistration: " << p.dump() << std::endl;
  } catch (const std::exception & e) {
    std::cerr << "unregister_message failed: " << e.what() << std::endl;
  }
}

}  // namespace mqtt_teleop
