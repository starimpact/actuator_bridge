#pragma once
#include <string>
#include <vector>
#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include "actuator_bridge/types.h"

namespace actuator_bridge {

struct ActuatorSpec {
  std::string name;     // logical name
  std::string vendor;   // "dm" | "robstride" | other
  int id;               // protocol id on the bus
  std::string model;    // optional model name, used to pick limits
};

struct Limits {
  float p_min{-12.5f}, p_max{12.5f};
  float v_min{-45.0f}, v_max{45.0f};
  float t_min{-20.0f}, t_max{20.0f};
  float kp_min{0.0f},  kp_max{500.0f};
  float kd_min{0.0f},  kd_max{5.0f};
};

class DriverBase {
 public:
  virtual ~DriverBase() = default;

  virtual std::string vendor() const = 0;

  // Map high-level command to one or more CAN frames.
  virtual void encodeCommand(const ActuatorCommand& cmd, std::vector<can_msgs::Frame>& out) = 0;

  // Try parse feedback frame, return true if it belongs to this driver and out is filled.
  virtual bool parseFeedback(const can_msgs::Frame& in, ActuatorFeedback& out) = 0;

  // Convenience helpers
  virtual void makeEnable(int id, std::vector<can_msgs::Frame>& out) = 0;
  virtual void makeDisable(int id, std::vector<can_msgs::Frame>& out) = 0;
  virtual void makeZero(int id, std::vector<can_msgs::Frame>& out) = 0;
};

}  // namespace actuator_bridge
