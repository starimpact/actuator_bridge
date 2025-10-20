#include "actuator_bridge/drivers/robstride_driver.h"
#include "actuator_bridge/driver_registry.h"
#include <cstring>

namespace {
constexpr float P_MIN = -12.57f, P_MAX = 12.57f;
constexpr float V_MIN = -50.0f , V_MAX = 50.0f;
constexpr float KP_MIN = 0.0f  , KP_MAX = 500.0f;
constexpr float KD_MIN = 0.0f  , KD_MAX = 5.0f;
constexpr float T_MIN = -5.5f  , T_MAX = 5.5f;

inline int float_to_uint(float x,float x_min,float x_max,int bits)
{
  float span = x_max - x_min;
  float offset = x_min;
  if(x > x_max) x = x_max;
  else if(x < x_min) x = x_min;
  return (int) ((x - offset)*((float)((1<<bits)-1))/span);
}

inline float byte_to_float(const uint8_t* bytedata){
  uint32_t data = (uint32_t)bytedata[7]<<24 | (uint32_t)bytedata[6]<<16 | (uint32_t)bytedata[5]<<8 | (uint32_t)bytedata[4];
  float f; std::memcpy(&f, &data, 4); return f;
}
}

namespace actuator_bridge {

void RobStrideDriver::encodeCommand(const ActuatorCommand& cmd, std::vector<can_msgs::Frame>& out) {
  // Motion control frame (extended) - follow RobStrite_Motor_move_control packing
  can_msgs::Frame f;
  f.is_extended = true;
  f.is_rtr = false;
  f.dlc = 8;
  const uint32_t Communication_Type_MotionControl = 0x01;
  // torque encoded into middle 16 bits in the RobStrite implementation; keep same packing
  uint32_t id = (Communication_Type_MotionControl << 24) |
                ((uint32_t)float_to_uint(cmd.torque, T_MIN, T_MAX, 16) << 8) |
                (uint32_t)(cmd.id & 0xFF);
  f.id = id;
  f.data.fill(0);
  Limits lim = getLimits(cmd.id);
  uint16_t p = (uint16_t)float_to_uint(cmd.pos, lim.p_min, lim.p_max, 16);
  uint16_t v = (uint16_t)float_to_uint(cmd.vel, lim.v_min, lim.v_max, 16);
  uint16_t kp = (uint16_t)float_to_uint(cmd.kp, lim.kp_min, lim.kp_max, 16);
  uint16_t kd = (uint16_t)float_to_uint(cmd.kd, lim.kd_min, lim.kd_max, 16);
  f.data[0] = (p >> 8) & 0xFF;
  f.data[1] = p & 0xFF;
  f.data[2] = (v >> 8) & 0xFF;
  f.data[3] = v & 0xFF;
  f.data[4] = (kp >> 8) & 0xFF;
  f.data[5] = kp & 0xFF;
  f.data[6] = (kd >> 8) & 0xFF;
  f.data[7] = kd & 0xFF;

  // Print CAN id and data similar to RobStrite implementation for debugging
  // printf("CAN ID: 0x%X, Data: ", f.id);
  // for (int i = 0; i < f.dlc; ++i) {
  //   printf("0x%02X ", f.data[i]);
  // }
  // printf("\n");

  out.push_back(f);
}

bool RobStrideDriver::parseFeedback(const can_msgs::Frame& in, ActuatorFeedback& out) {
  if (!in.is_extended) return false; // robstride uses extended
  // feedback type 0x02 in higher bits
  const int type = (int)((in.id & 0x3F000000) >> 24);
  if (type != 2) return false;
  out.vendor = "robstride";
  out.id = (int)((in.id & 0xFF00) >> 8);
  // parse data
  auto u16 = [&](int idx_hi, int idx_lo){ return (uint16_t)((in.data[idx_hi] << 8) | in.data[idx_lo]); };
  uint16_t p_hex = u16(0,1);
  uint16_t v_hex = u16(2,3);
  uint16_t t_hex = u16(4,5);
  Limits lim = getLimits(out.id);
  out.pos = (float)( (lim.p_max - lim.p_min) * (float)p_hex / ((1<<16)-1) + lim.p_min );
  out.vel = (float)( (lim.v_max - lim.v_min) * (float)v_hex / ((1<<16)-1) + lim.v_min );
  out.torque = (float)( (lim.t_max - lim.t_min) * (float)t_hex / ((1<<16)-1) + lim.t_min );
  out.temp1 = 0.1f * u16(6,7);
  out.temp2 = 0.0f;
  out.state = (int)((in.id & 0xC00000) >> 22); // pattern
  out.error = (uint32_t)((in.id & 0x3F0000) >> 16);
  out.raw = in;
  return true;
}

void RobStrideDriver::makeEnable(int id, std::vector<can_msgs::Frame>& out) {
  const uint32_t Communication_Type_MotorEnable = 0x03;
  can_msgs::Frame f; f.is_extended=true; f.is_rtr=false; f.dlc=8; f.id=(Communication_Type_MotorEnable<<24) | (0x00<<8) | (uint32_t)(id & 0xFF); f.data.fill(0); out.push_back(f);
}
void RobStrideDriver::makeDisable(int id, std::vector<can_msgs::Frame>& out) {
  const uint32_t Communication_Type_MotorStop = 0x04;
  can_msgs::Frame f; f.is_extended=true; f.is_rtr=false; f.dlc=8; f.id=(Communication_Type_MotorStop<<24) | (0x00<<8) | (uint32_t)(id & 0xFF); f.data.fill(0); out.push_back(f);
}
void RobStrideDriver::makeZero(int id, std::vector<can_msgs::Frame>& out) {
  const uint32_t Communication_Type_SetPosZero = 0x06;
  can_msgs::Frame f; f.is_extended=true; f.is_rtr=false; f.dlc=8; f.id=(Communication_Type_SetPosZero<<24) | (0x00<<8) | (uint32_t)(id & 0xFF); f.data.fill(0); f.data[0]=1; out.push_back(f);
}

} // namespace actuator_bridge

// static registrar to auto-register RobStrideDriver
namespace {
struct _rob_reg {
  _rob_reg() {
    actuator_bridge::DriverRegistry::instance().registerFactory("robstride", [](){
      return std::make_shared<actuator_bridge::RobStrideDriver>();
    });
  }
} _rob_reg_instance;
}
