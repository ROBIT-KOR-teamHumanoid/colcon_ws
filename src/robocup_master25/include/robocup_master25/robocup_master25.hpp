#ifndef ROBOCUP_MASTER25_HPP
#define ROBOCUP_MASTER25_HPP

#include <iostream>
#include <rclcpp/parameter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "point.hpp"

// imu
#include "humanoid_interfaces/msg/imu_msg.hpp"

// vision
#include "humanoid_interfaces/msg/master2vision25.hpp"
#include "humanoid_interfaces/msg/robocupvision25.hpp"

// localization
#include "humanoid_interfaces/msg/master2localization25.hpp"
#include "humanoid_interfaces/msg/robocuplocalization25.hpp"

// ikwalk
#include "humanoid_interfaces/msg/ik_end_msg.hpp"
#include "humanoid_interfaces/msg/master2_ik_msg.hpp"

// gamecontroller
#include "RoboCupGameControlData.hpp"
#include "humanoid_interfaces/msg/gamecontroldata.hpp"
#include "humanoid_interfaces/msg/gamecontrolreturndata.hpp"

// udp

#include "humanoid_interfaces/msg/master2udp.hpp"
#include "humanoid_interfaces/msg/udp2master.hpp"

// pid
#include "humanoid_interfaces/msg/pidtuning.hpp"

// motion
#include "/home/robit/colcon_ws/src/robot_config.h"
#include "humanoid_interfaces/msg/motion_operator.hpp"

namespace robocup_master25 {
class MasterRcko {
  // parameters
 public:
  double kp;
  double kd;
  double ki;

  int FRONT_MAX;
  int REAR_MAX;
  int RIGHT_MAX;
  int LEFT_MAX;
  int R_YAW_MAX;
  int L_YAW_MAX;
  int X_MIN;
  int Y_MIN;
  int ROUND_Y;
  int ROUND_YAW_MIN;

  int In;
  int Out;
  int Back;
  int Front;

  // interface
 public:
  humanoid_interfaces::msg::ImuMsg imu;
  humanoid_interfaces::msg::Robocupvision25 vision;
  humanoid_interfaces::msg::IkEndMsg ikEnd;
  humanoid_interfaces::msg::Gamecontroldata gameControlData;
  humanoid_interfaces::msg::Robocuplocalization25 local;
  humanoid_interfaces::msg::Master2localization25 master2local;
  humanoid_interfaces::msg::Master2udp master2udp;
  humanoid_interfaces::msg::Udp2master udp[4];
  humanoid_interfaces::msg::MotionOperator motion;
  humanoid_interfaces::msg::MotionOperator motionEnd;
  humanoid_interfaces::msg::Gamecontrolreturndata gameControlReturnData;
  humanoid_interfaces::msg::Master2IkMsg ik;
  humanoid_interfaces::msg::Master2vision25 master2vision;
  humanoid_interfaces::msg::Pidtuning pid;
};
}  // namespace robocup_master25
#endif
