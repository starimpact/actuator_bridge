#pragma once
#include "actuator_bridge/driver_base.h"
#include <unordered_map>

namespace actuator_bridge {

class RobStrideDriver : public DriverBase {
 public:
  RobStrideDriver() = default;
  std::string vendor() const override { return "robstride"; }
  void encodeCommand(const ActuatorCommand& cmd, std::vector<can_msgs::Frame>& out) override;
  bool parseFeedback(const can_msgs::Frame& in, ActuatorFeedback& out) override;
  void makeEnable(int id, std::vector<can_msgs::Frame>& out) override;
  void makeDisable(int id, std::vector<can_msgs::Frame>& out) override;
  void makeZero(int id, std::vector<can_msgs::Frame>& out) override;
 private:
  // For future per-actuator limits, not used yet in parsing (only in encode)
  std::unordered_map<int, Limits> limits_;
 public:
  void setLimits(int id, const Limits& lim) { limits_[id]=lim; }
  Limits getLimits(int id) const { auto it=limits_.find(id); return it==limits_.end()?Limits{-12.57f,12.57f,-50.0f,50.0f,-5.5f,5.5f,0.0f,500.0f,0.0f,5.0f}:it->second; }
};

} // namespace actuator_bridge
