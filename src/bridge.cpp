#include "actuator_bridge/bridge.h"
#include <boost/bind.hpp>

namespace actuator_bridge {

ActuatorBridge::ActuatorBridge(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh) {
}

bool ActuatorBridge::loadFromParams(){
  XmlRpc::XmlRpcValue buses;
  if (!(pnh_.getParam("buses", buses) && buses.getType()==XmlRpc::XmlRpcValue::TypeArray)) {
    ROS_ERROR("~buses param must be an array");
    return false;
  }
  return buildFromBusesConfig(buses);
}

bool ActuatorBridge::loadFromConfig(const XmlRpc::XmlRpcValue& buses){
  if (buses.getType()!=XmlRpc::XmlRpcValue::TypeArray) return false;
  return buildFromBusesConfig(buses);
}

int ActuatorBridge::getOrCreateBus(const std::string& vendor, const std::string& rx, const std::string& tx){
  const std::string key = vendor + "|" + rx + "|" + tx;
  auto it = bus_index_by_key_.find(key);
  if (it != bus_index_by_key_.end()) return it->second;
  Bus newbus; newbus.rx_topic=rx; newbus.tx_topic=tx;
  if (vendor=="dm") newbus.driver = std::make_shared<DMDriver>(vendor);
  else if (vendor=="robstride") newbus.driver = std::make_shared<RobStrideDriver>(vendor);
  else throw std::runtime_error("unsupported vendor: "+vendor);
  int idx = (int)buses_.size();
  newbus.pub = nh_.advertise<can_msgs::Frame>(tx, 100);
  newbus.sub = nh_.subscribe<can_msgs::Frame>(rx, 100, boost::bind(&ActuatorBridge::onCanRx, this, _1, idx));
  buses_.push_back(std::move(newbus));
  bus_index_by_key_[key] = idx;
  return idx;
}

bool ActuatorBridge::buildFromBusesConfig(const XmlRpc::XmlRpcValue& buses){
  // Clear existing
  buses_.clear(); bus_index_by_key_.clear(); name_to_bus_.clear(); name_to_id_.clear();

  for (int i=0;i<buses.size();++i) {
    auto& b = buses[i];
    if (b.getType()!=XmlRpc::XmlRpcValue::TypeStruct) continue;
    std::string vendor = static_cast<std::string>(b["vendor"]);

    // model limits table
    std::unordered_map<std::string, Limits> model_limits;
    if (b.hasMember("models") && b["models"].getType()==XmlRpc::XmlRpcValue::TypeStruct) {
      auto& models = b["models"];
      for (auto it=models.begin(); it!=models.end(); ++it) {
        std::string mname = static_cast<std::string>(it->first);
        auto& m = it->second; if (m.getType()!=XmlRpc::XmlRpcValue::TypeStruct) continue;
        Limits lim; auto load=[&](const char* k, float& d){ if (m.hasMember(k)) d = static_cast<double>(m[k]); };
        load("p_min", lim.p_min); load("p_max", lim.p_max);
        load("v_min", lim.v_min); load("v_max", lim.v_max);
        load("t_min", lim.t_min); load("t_max", lim.t_max);
        load("kp_min", lim.kp_min); load("kp_max", lim.kp_max);
        load("kd_min", lim.kd_min); load("kd_max", lim.kd_max);
        model_limits[mname]=lim;
      }
    }

    if (b.hasMember("actuators") && b["actuators"].getType()==XmlRpc::XmlRpcValue::TypeArray) {
      for (int j=0;j<b["actuators"].size();++j) {
        auto& a = b["actuators"][j]; if (a.getType()!=XmlRpc::XmlRpcValue::TypeStruct) continue;
        std::string name = static_cast<std::string>(a["name"]);
        int id = static_cast<int>(a["id"]);
        std::string model = a.hasMember("model") ? static_cast<std::string>(a["model"]) : std::string();
        std::string can_device = a.hasMember("can_device") ? static_cast<std::string>(a["can_device"]) : std::string();
        std::string rx_topic = a.hasMember("rx_topic") ? static_cast<std::string>(a["rx_topic"]) : std::string();
        std::string tx_topic = a.hasMember("tx_topic") ? static_cast<std::string>(a["tx_topic"]) : std::string();
        if (!can_device.empty()) {
          if (rx_topic.empty()) rx_topic = "/"+can_device+"_rx";
          if (tx_topic.empty()) tx_topic = "/"+can_device+"_tx";
        }
        if (rx_topic.empty() || tx_topic.empty()) {
          ROS_WARN("actuator %s missing rx/tx_topic and can_device, skip", name.c_str());
          continue;
        }
        int bus_idx = getOrCreateBus(vendor, rx_topic, tx_topic);
        ActuatorSpec s; s.name=name; s.vendor=vendor; s.id=id; s.model=model;
        buses_[bus_idx].actuators.push_back(s);
        name_to_bus_[name] = bus_idx; name_to_id_[name] = id;

        // limits
        Limits lim; bool has_limits=false;
        if (!model.empty()) { auto it=model_limits.find(model); if (it!=model_limits.end()) { lim=it->second; has_limits=true; } }
        auto apply_inline=[&](const char* k, float& d){ if (a.hasMember(k)) { d=static_cast<double>(a[k]); has_limits=true; } };
        apply_inline("p_min", lim.p_min); apply_inline("p_max", lim.p_max);
        apply_inline("v_min", lim.v_min); apply_inline("v_max", lim.v_max);
        apply_inline("t_min", lim.t_min); apply_inline("t_max", lim.t_max);
        apply_inline("kp_min", lim.kp_min); apply_inline("kp_max", lim.kp_max);
        apply_inline("kd_min", lim.kd_min); apply_inline("kd_max", lim.kd_max);
        if (has_limits) {
          if (auto d = std::dynamic_pointer_cast<DMDriver>(buses_[bus_idx].driver)) d->setLimits(id, lim);
          if (auto r = std::dynamic_pointer_cast<RobStrideDriver>(buses_[bus_idx].driver)) r->setLimits(id, lim);
        }
      }
    }
  }

  return true;
}

void ActuatorBridge::enableAll(){
  forEachActuatorSend([](DriverBase& d, int id, std::vector<can_msgs::Frame>& out){ d.makeEnable(id, out); });
}
void ActuatorBridge::disableAll(){
  forEachActuatorSend([](DriverBase& d, int id, std::vector<can_msgs::Frame>& out){ d.makeDisable(id, out); });
}

bool ActuatorBridge::sendCommand(const ActuatorCommand& cmd){
  // print ActuatorCommand
  // printf("Routing command: name=%s id=%d pos=%.3f vel=%.3f tq=%.3f kp=%.3f kd=%.3f\n",
  //        cmd.name.c_str(), cmd.id, cmd.pos, cmd.vel, cmd.torque, cmd.kp, cmd.kd);

  int bus_idx=-1; int id=-1;
  if (!cmd.name.empty()) {
    // show name_to_bus_
    // for (auto& pair : name_to_bus_) {
    //   printf("Mapping: %s -> bus %d\n", pair.first.c_str(), pair.second);
    // }
    // printf("Routing command for actuator name=%s\n", cmd.name.c_str());
    auto it = name_to_bus_.find(cmd.name);
    if (it!=name_to_bus_.end()) { bus_idx = it->second; id = name_to_id_[cmd.name]; }
  } else if (cmd.id>=0) {
    // route by id within each bus; choose the first bus that has a matching actuator id
    for (size_t i=0;i<buses_.size() && bus_idx<0;++i) {
      for (const auto& s : buses_[i].actuators) { if (s.id==cmd.id) { bus_idx=(int)i; id=cmd.id; break; } }
    }
  }
  if (bus_idx<0) { ROS_WARN_THROTTLE(1.0, "Actuator command routing failed (name: %s)", cmd.name.c_str()); return false; }
  auto& bus = buses_[bus_idx];
  std::vector<can_msgs::Frame> frames; ActuatorCommand c = cmd; c.id = id; bus.driver->encodeCommand(c, frames);
  // printf("Sending command to actuator name=%s id=%d via bus %d (%s->%s), %zu frames",
        //  cmd.name.c_str(), id, bus_idx, bus.rx_topic.c_str(), bus.tx_topic.c_str(), frames.size());
  // print the first frame id and data
  // if (!frames.empty()) {
  //   const auto& f = frames[0];
  //   printf(", first frame ID=0x%X Data=", f.id);
  //   for (int i = 0; i < f.dlc; ++i) {
  //     printf("0x%02X ", f.data[i]);
  //   }
  // }
  // printf("\n");
  for (auto& f: frames) bus.pub.publish(f);
  return true;
}

template<typename Fn>
void ActuatorBridge::forEachActuatorSend(Fn&& fn){
  for (auto& bus : buses_) {
    for (const auto& s : bus.actuators) {
      std::vector<can_msgs::Frame> frames; fn(*bus.driver, s.id, frames); for (auto& f: frames) bus.pub.publish(f);
    }
  }
}

void ActuatorBridge::onCanRx(const can_msgs::Frame::ConstPtr& msg, int bus_idx){
  auto& bus = buses_[bus_idx];
  ActuatorFeedback fb; if (bus.driver->parseFeedback(*msg, fb)) {
    for (const auto& s : bus.actuators) if (s.id==fb.id) { fb.name=s.name; break; }
    // update snapshot
    if (!fb.name.empty()) {
      std::lock_guard<std::mutex> lock(fb_mutex_);
      // printf("Received feedback from actuator name=%s id=%d via bus %d (%s), pos=%.3f vel=%.3f tq=%.3f\n",
      //        fb.name.c_str(), fb.id, bus_idx, bus.rx_topic.c_str(), fb.pos, fb.vel, fb.torque);
      last_feedback_by_name_[fb.name] = fb;
    }
    if (fb_cb_) fb_cb_(fb);
  }
}

} // namespace actuator_bridge
