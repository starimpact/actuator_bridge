#pragma once

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <mutex>

#include <ros/ros.h>
#include <xmlrpcpp/XmlRpcValue.h>
#include <can_msgs/Frame.h>

#include "actuator_bridge/types.h"
#include "actuator_bridge/driver_base.h"
#include "actuator_bridge/drivers/dm_driver.h"
#include "actuator_bridge/drivers/robstride_driver.h"

namespace actuator_bridge {

// Public reusable bridge that can be embedded in any node.
class ActuatorBridge {
 public:
  explicit ActuatorBridge(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~ActuatorBridge() { 
    ROS_INFO("Shutting down actuator bridge...");
    disableAll(); 
  }

  // Load configuration from ROS param (~buses). Returns false if invalid.
  bool loadFromParams();
  // Or load from a provided XmlRpcValue (array with vendor groups)
  bool loadFromConfig(const XmlRpc::XmlRpcValue& buses);

  // Control helpers
  void enableAll();
  void disableAll();

  // Send a high-level command (by name preferred). Returns false if routing failed.
  bool sendCommand(const ActuatorCommand& cmd);

  // Allow consumer to receive parsed feedback via callback in addition to topic publication.
  using FeedbackCallback = std::function<void(const ActuatorFeedback&)>;
  void setFeedbackCallback(FeedbackCallback cb) { fb_cb_ = std::move(cb); }

  // Direct memory accessors for latest feedback (per actuator name)
  // Thread-safe snapshot copy of all feedback
  std::unordered_map<std::string, ActuatorFeedback> getFeedbackSnapshot() const {
    std::lock_guard<std::mutex> lock(fb_mutex_);
    return last_feedback_by_name_;
  }
  // Thread-safe lookup of last feedback by name
  bool getLastFeedback(const std::string& name, ActuatorFeedback& out) const {
    std::lock_guard<std::mutex> lock(fb_mutex_);
    auto it = last_feedback_by_name_.find(name);
    if (it == last_feedback_by_name_.end()) return false;
    out = it->second; return true;
  }

 private:
  struct Bus {
    std::string rx_topic;
    std::string tx_topic;
    ros::Subscriber sub;
    ros::Publisher pub;
    std::vector<ActuatorSpec> actuators;
    std::shared_ptr<DriverBase> driver;
  };

  template<typename Fn>
  void forEachActuatorSend(Fn&& fn);

  void onCanRx(const can_msgs::Frame::ConstPtr& msg, int bus_idx);

  // internal utils
  int getOrCreateBus(const std::string& vendor, const std::string& rx, const std::string& tx);
  bool buildFromBusesConfig(const XmlRpc::XmlRpcValue& buses);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  std::vector<Bus> buses_;
  std::unordered_map<std::string,int> bus_index_by_key_;
  std::unordered_map<std::string,int> name_to_bus_;
  std::unordered_map<std::string,int> name_to_id_;
  FeedbackCallback fb_cb_;

  // Latest feedback snapshot keyed by actuator name
  std::unordered_map<std::string, ActuatorFeedback> last_feedback_by_name_;
  mutable std::mutex fb_mutex_;
};

} // namespace actuator_bridge
