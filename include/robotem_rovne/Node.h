/**
 * Copyright (c) 2024 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/robotem_rovne/graphs/contributors.
 */

#pragma once

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <memory>

#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <mp-units/systems/si/si.h>
#include <mp-units/systems/angular/angular.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

using namespace mp_units;
using mp_units::si::unit_symbols::s;
using mp_units::si::unit_symbols::m;
using mp_units::angular::unit_symbols::deg;
using mp_units::angular::unit_symbols::rad;

namespace t07
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class Node : public rclcpp::Node
{
public:
   Node();
  ~Node();

private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _cmd_vel_sub;
  quantity<m/s> _linear_vel_target;
  quantity<rad/s> _yaw_angular_vel_target;
  void init_cmd_vel_sub();

  rclcpp::QoS _imu_qos_profile;
  rclcpp::SubscriptionOptions _imu_sub_options;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _imu_sub;
  quantity<rad> _yaw_actual;
  quantity<rad/s> _yaw_angular_vel_actual;
  void init_imu_sub();


  rclcpp::QoS _motor_left_qos_profile;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _motor_left_pub;
  void init_motor_left_pub();
  void pub_motor_left(quantity<m/s> const velocity);

  rclcpp::QoS _motor_right_qos_profile;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _motor_right_pub;
  void init_motor_right_pub();
  void pub_motor_right(quantity<m/s> const velocity);


  static std::chrono::milliseconds constexpr CTRL_LOOP_RATE{50};
  rclcpp::TimerBase::SharedPtr _ctrl_loop_timer;
  void init_ctrl_loop();
  void ctrl_loop();
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* t07 */
