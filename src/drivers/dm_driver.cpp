#include "actuator_bridge/drivers/dm_driver.h"
#include "actuator_bridge/driver_registry.h"
#include <algorithm>

namespace {
// default constants (can be overridden per actuator via limits_)
constexpr float KP_MAX_D = 500.0f, KP_MIN_D = 0.0f;
constexpr float KD_MAX_D = 5.0f,   KD_MIN_D = 0.0f;
constexpr float P_MAX_D = 12.5f,   P_MIN_D = -12.5f;
constexpr float V_MAX_D = 45.0f,   V_MIN_D = -45.0f;
constexpr float T_MAX_D = 20.0f,   T_MIN_D = -20.0f;

inline int float_to_uint(float x,float x_min,float x_max,int bits) {
  float span = x_max - x_min;
  float offset = x_min;
  if(x > x_max) x = x_max;
  else if(x < x_min) x = x_min;
  return (int) ((x - offset)*((float)((1<<bits)-1))/span);
}

inline float uint_to_float(uint16_t x, float x_max, float x_min, int bits){
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x)*span/((float)((1<<bits)-1)) + offset;
}
}

namespace actuator_bridge {

void DMDriver::encodeCommand(const ActuatorCommand& cmd, std::vector<can_msgs::Frame>& out) {
  can_msgs::Frame f; f.is_extended = false; f.is_rtr = false; f.dlc = 8; f.id = cmd.id;
  Limits lim{P_MIN_D,P_MAX_D,V_MIN_D,V_MAX_D,T_MIN_D,T_MAX_D,KP_MIN_D,KP_MAX_D,KD_MIN_D,KD_MAX_D};
  if (auto it = limits_.find(cmd.id); it != limits_.end()) lim = it->second;
  uint16_t pos_tmp = float_to_uint(cmd.pos, lim.p_min, lim.p_max, 16);
  uint16_t vel_tmp = float_to_uint(cmd.vel, lim.v_min, lim.v_max, 12);
  uint16_t kp_tmp  = float_to_uint(cmd.kp,  lim.kp_min, lim.kp_max, 12);
  uint16_t kd_tmp  = float_to_uint(cmd.kd,  lim.kd_min, lim.kd_max, 12);
  uint16_t tor_tmp = float_to_uint(cmd.torque, lim.t_min, lim.t_max, 12);
  f.data.fill(0);
  f.data[0] = (pos_tmp >> 8);
  f.data[1] = (pos_tmp & 0xFF);
  f.data[2] = ((vel_tmp >> 4) & 0xFF);
  f.data[3] = ((((vel_tmp & 0xF) << 4) & 0xFF) | ((kp_tmp >> 8) & 0xFF));
  f.data[4] = (kp_tmp & 0xFF);
  f.data[5] = ((kd_tmp >> 4) & 0xFF);
  f.data[6] = ((((kd_tmp & 0xF) << 4) & 0xFF) | ((tor_tmp >> 8) & 0xFF));
  f.data[7] = (tor_tmp & 0xFF);
  out.push_back(f);
}

bool DMDriver::parseFeedback(const can_msgs::Frame& in, ActuatorFeedback& out) {
  if (in.is_extended) return false; // dm uses std frame
  int id = (int)(in.data[0] & 0x0F); // payload low 4 bits
  // basic sanity
  if (in.dlc != 8) return false;
  out.vendor = "dm";
  out.id = id;
  // temps in last 2 bytes
  uint16_t pos_hex = (uint16_t)(in.data[2] | (in.data[1] << 8));
  uint16_t vel_hex = (uint16_t)((in.data[4] >> 4) | (in.data[3] << 4));
  uint16_t t_hex   = (uint16_t)((in.data[5] | (in.data[4] & 0x0F) << 8));
  Limits lim{P_MIN_D,P_MAX_D,V_MIN_D,V_MAX_D,T_MIN_D,T_MAX_D,KP_MIN_D,KP_MAX_D,KD_MIN_D,KD_MAX_D};
  if (auto it = limits_.find(id); it != limits_.end()) lim = it->second;
  out.pos = uint_to_float(pos_hex, lim.p_max, lim.p_min, 16);
  out.vel = uint_to_float(vel_hex, lim.v_max, lim.v_min, 12);
  out.torque = uint_to_float(t_hex, lim.t_max, lim.t_min, 12);
  out.temp1 = in.data[6];
  out.temp2 = in.data[7];
  out.state = (uint16_t)(in.data[0]) >> 4;
  out.raw = in;
  return true;
}

void DMDriver::makeEnable(int id, std::vector<can_msgs::Frame>& out) {
  can_msgs::Frame f; f.is_extended=false; f.is_rtr=false; f.dlc=8; f.id=id; f.data.fill(0xFF); f.data[7]=0xFC; out.push_back(f);
}
void DMDriver::makeDisable(int id, std::vector<can_msgs::Frame>& out) {
  can_msgs::Frame f; f.is_extended=false; f.is_rtr=false; f.dlc=8; f.id=id; f.data.fill(0xFF); f.data[7]=0xFD; out.push_back(f);
}
void DMDriver::makeZero(int id, std::vector<can_msgs::Frame>& out) {
  can_msgs::Frame f; f.is_extended=false; f.is_rtr=false; f.dlc=8; f.id=id; f.data.fill(0xFF); f.data[7]=0xFE; out.push_back(f);
}

} // namespace actuator_bridge

// static registrar to auto-register DMDriver
namespace {
struct _dm_reg {
  _dm_reg() {
    actuator_bridge::DriverRegistry::instance().registerFactory("dm", [](){
      return std::make_shared<actuator_bridge::DMDriver>();
    });
  }
} _dm_reg_instance;
}
