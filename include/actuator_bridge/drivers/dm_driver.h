#pragma once
#include "actuator_bridge/driver_base.h"
#include <unordered_map>

namespace actuator_bridge {

class DMDriver : public DriverBase {
 public:
  DMDriver(std::string can_name) : can_name_(std::move(can_name)) {}
  std::string vendor() const override { return "dm"; }
  void encodeCommand(const ActuatorCommand& cmd, std::vector<can_msgs::Frame>& out) override;
  bool parseFeedback(const can_msgs::Frame& in, ActuatorFeedback& out) override;
  void makeEnable(int id, std::vector<can_msgs::Frame>& out) override;
  void makeDisable(int id, std::vector<can_msgs::Frame>& out) override;
  void makeZero(int id, std::vector<can_msgs::Frame>& out) override;
  // per-actuator limits
  void setLimits(int id, const Limits& lim) { limits_[id] = lim; }
  Limits getLimits(int id) const {
    auto it = limits_.find(id);
    if (it != limits_.end()) return it->second;
    return Limits{};
  }
 private:
  std::string can_name_;
  std::unordered_map<int, Limits> limits_;
};

} // namespace actuator_bridge
