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

#include <mp-units/systems/si/si.h>
#include <mp-units/systems/angular/angular.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

using namespace mp_units;
using mp_units::si::unit_symbols::m;
using mp_units::si::unit_symbols::mm;
using mp_units::si::unit_symbols::s;
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

  static std::chrono::milliseconds constexpr CTRL_LOOP_RATE{50};
  rclcpp::TimerBase::SharedPtr _ctrl_loop_timer;
  void init_ctrl_loop();
  void ctrl_loop();
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* t07 */
