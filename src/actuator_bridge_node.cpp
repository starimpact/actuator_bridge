#include <ros/ros.h>
#include <cmath>
#include <string>
#include "actuator_bridge/bridge.h"
#include <csignal>

// static volatile std::sig_atomic_t g_shutdown_requested = 0;
// static void handle_signal(int) { g_shutdown_requested = 1; }

int main(int argc, char** argv){
  ros::init(argc, argv, "actuator_bridge_node");
  ros::NodeHandle nh; ros::NodeHandle pnh("~");
  try {
    actuator_bridge::ActuatorBridge bridge(nh, pnh);
    if (!bridge.loadFromParams()) return 1;

    bool auto_enable=false; pnh.param("auto_enable", auto_enable, false);
    if (auto_enable) {
      ros::Duration(0.5).sleep(); // wait for everything to come up
      bridge.enableAll();
      // bridge.disableAll(); // test disable right after enable
    }
    // return 0;
    // Optional: periodically print a snapshot of feedback (memory access, no topics)
    bool print_feedback=false; pnh.param("print_feedback", print_feedback, false);
    double print_rate=2.0;    pnh.param("print_rate", print_rate, 2.0);
    ros::Timer print_timer;
    if (print_feedback && print_rate > 0.0) {
      print_timer = nh.createTimer(ros::Duration(1.0/std::max(1e-3, print_rate)),
        [&](const ros::TimerEvent&){
          auto snap = bridge.getFeedbackSnapshot();
          for (const auto& kv : snap) {
            const auto& fb = kv.second;
            ROS_INFO("fb name=%s id=%d vendor=%s pos=%.3f vel=%.3f tq=%.3f",
                     fb.name.c_str(), fb.id, fb.vendor.c_str(), fb.pos, fb.vel, fb.torque);
          }
        });
    }

    // Optional demo: periodically send a command using library API (no topics)
    bool demo_enable=false; pnh.param("demo_enable", demo_enable, false);
    std::string demo_mode="hold"; pnh.param("demo_mode", demo_mode, demo_mode);
    std::string demo_name; pnh.param("demo_name", demo_name, demo_name); // actuator name to route
    double demo_rate=100.0; pnh.param("demo_rate", demo_rate, 100.0);
    double freq=0.5; pnh.param("demo_freq", freq, 0.5);    // sine frequency (Hz)

    double amp=0.0; pnh.param("demo_amp", amp, 0.0);       // pos amplitude (rad)
    double vel=0.1; pnh.param("demo_vel", vel, 1.0);
    double tq =0.0; pnh.param("demo_torque", tq, 0.0);
    double kp =2.76; pnh.param("demo_kp", kp, 0.0);
    double kd = 0.1760; pnh.param("demo_kd", kd, 0.17);

    ros::Timer demo_timer;
    // printf("Demo parameters: mode=%s name=%s rate=%.1f amp=%.3f freq=%.3f vel=%.3f tq=%.3f kp=%.3f kd=%.3f\n",
          //  demo_mode.c_str(), demo_name.c_str(), demo_rate, amp, freq, vel, tq, kp, kd);
    if (demo_enable && !demo_name.empty() && demo_rate > 0.0) {
      const double dt = 1.0/std::max(1e-3, demo_rate);
      const double two_pi = 2.0 * M_PI;
      ros::Time t0 = ros::Time::now();
      printf("Starting demo mode '%s' for actuator '%s' at %.1f Hz\n", demo_mode.c_str(), demo_name.c_str(), demo_rate);
      demo_timer = nh.createTimer(ros::Duration(dt),
        [&, t0, two_pi, demo_mode, demo_name, amp, freq, vel, tq, kp, kd](const ros::TimerEvent& ev){
          actuator_bridge::ActuatorCommand cmd;
          cmd.name = demo_name;
          cmd.pos = static_cast<float>(amp);
          cmd.vel = static_cast<float>(vel);
          cmd.torque = static_cast<float>(tq);
          cmd.kp = static_cast<float>(kp);
          cmd.kd = static_cast<float>(kd);
          
          // if (demo_mode == "sine") {
          //   double t = (ev.current_real - t0).toSec();
          //   cmd.pos = static_cast<float>(amp * std::sin(two_pi * freq * t));
          // } else { // hold
          //   cmd.pos = static_cast<float>(amp);
          // }
          // printf("Demo sending command to %s: pos=%.3f vel=%.3f tq=%.3f kp=%.3f kd=%.3f\n",
          //        cmd.name.c_str(), cmd.pos, cmd.vel, cmd.torque, cmd.kp, cmd.kd);
          (void)bridge.sendCommand(cmd);
        });
    }

    // Optional: periodically query and print a specific motor status by name (memory read)
    std::string status_name; pnh.param("status_name", status_name, std::string());
    double status_rate=0.0; pnh.param("status_rate", status_rate, 0.0);
    ros::Timer status_timer;
    if (!status_name.empty() && status_rate > 0.0) {
      status_timer = nh.createTimer(ros::Duration(1.0/std::max(1e-3, status_rate)),
        [&, status_name](const ros::TimerEvent&){
          actuator_bridge::ActuatorFeedback fb;
          if (bridge.getLastFeedback(status_name, fb)) {
            ROS_INFO("status name=%s id=%d pos=%.3f vel=%.3f tq=%.3f temp=%.1f/%.1f state=%d err=%u",
                     fb.name.c_str(), fb.id, fb.pos, fb.vel, fb.torque, fb.temp1, fb.temp2, fb.state, fb.error);
          } else {
            ROS_WARN_THROTTLE(1.0, "no feedback yet for %s", status_name.c_str());
          }
        });
    }

    // Replace blocking spin with a loop so we can intercept signals and
    // call bridge.disableAll() while publishers are still valid.
    // signal(SIGINT, handle_signal);
    // signal(SIGTERM, handle_signal);
    // ros::Rate loop_rate(50);
    ros::spin();
    // while (ros::ok() && !g_shutdown_requested) {
    //   ros::spinOnce();
    //   loop_rate.sleep();
    // }
    // if (g_shutdown_requested) {
    //   ROS_INFO("Shutdown requested, disabling actuators before ROS shutdown");
    //   bridge.disableAll();
    //   // Now trigger normal ROS shutdown so other resources clean up
    //   ros::shutdown();
    // }
  } catch (const std::exception& e) {
    ROS_FATAL("actuator_bridge_node failed: %s", e.what());
    return 1;
  }
  return 0;
}
