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

#include <std_srvs/srv/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <robotem_rovne/srv/angular_target.hpp>

#include <mp-units/systems/si/si.h>
#include <mp-units/systems/angular/angular.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

using namespace mp_units;
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
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr _req_start_service_server;
  void init_req_start_service_server();

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr _req_stop_service_server;
  void init_req_stop_service_server();

  rclcpp::Service<robotem_rovne::srv::AngularTarget>::SharedPtr _req_set_angular_target_service_server;
  quantity<rad> _yaw_target;
  void init_req_set_angular_target_service_server();


  rclcpp::QoS _imu_qos_profile;
  rclcpp::SubscriptionOptions _imu_sub_options;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _imu_sub;
  quantity<rad> _yaw_actual;
  void init_imu_sub();


  rclcpp::QoS _motor_left_qos_profile;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _motor_left_pub;
  void init_motor_left_pub();

  rclcpp::QoS _motor_right_qos_profile;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _motor_right_pub;
  void init_motor_right_pub();


  static std::chrono::milliseconds constexpr CTRL_LOOP_RATE{50};
  rclcpp::TimerBase::SharedPtr _ctrl_loop_timer;
  void init_ctrl_loop();
  void ctrl_loop();

  enum class State { Stopped, Starting, Driving, Stopping };
  State _robot_state;
  void handle_Stopped();
  void handle_Starting();
  void handle_Driving();
  void handle_Stopping();
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* t07 */
