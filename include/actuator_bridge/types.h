#pragma once
#include <string>
#include <vector>
#include <cstdint>
#include <can_msgs/Frame.h>

namespace actuator_bridge {

struct ActuatorCommand {
  // routing
  std::string name;   // 优先按名称路由
  std::string vendor; // 可选，未使用
  int32_t id{-1};     // name 为空时回退按 id 路由
  // MIT 风格控制量
  float pos{0.0f};
  float vel{0.0f};
  float torque{0.0f};
  float kp{0.0f};
  float kd{0.0f};
  // 高级：原始帧（如需直通）
  can_msgs::Frame raw;
};

struct ActuatorFeedback {
  // 源信息
  std::string name;
  std::string vendor;
  int32_t id{0};
  // 状态
  float pos{0.0f};
  float vel{0.0f};
  float torque{0.0f};
  float temp1{0.0f};
  float temp2{0.0f};
  int32_t state{0};
  uint32_t error{0};
  // 调试：原始帧
  can_msgs::Frame raw;
};

} // namespace actuator_bridge
