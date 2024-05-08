/**
 * Copyright (c) 2024 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/robotem_rovne/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <robotem_rovne/Node.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace t07
{

/**************************************************************************************
 * CTOR/DTOR 
 **************************************************************************************/

Node::Node()
: rclcpp::Node("robotem_rovne_node")
{
  init_req_start_service_server();
  init_req_stop_service_server();
  init_req_set_angular_target_service_server();

  init_ctrl_loop();

  RCLCPP_INFO(get_logger(), "%s init complete.", get_name());
}

Node::~Node()
{
  RCLCPP_INFO(get_logger(), "%s shut down successfully.", get_name());
}

/**************************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 **************************************************************************************/

void Node::init_req_start_service_server()
{
  _req_start_service_server = create_service<std_srvs::srv::Empty>(
    "cmd_robot/start",
    [this](std_srvs::srv::Empty::Request::SharedPtr /* request */,
           std_srvs::srv::Empty::Response::SharedPtr /* response */)
    {
      RCLCPP_INFO(get_logger(), "start request received");
    });
}

void Node::init_req_stop_service_server()
{
  _req_stop_service_server = create_service<std_srvs::srv::Empty>(
    "cmd_robot/stop",
    [this](std_srvs::srv::Empty::Request::SharedPtr /* request */,
           std_srvs::srv::Empty::Response::SharedPtr /* response */)
    {
      RCLCPP_INFO(get_logger(), "stop request received");
    });
}

void Node::init_req_set_angular_target_service_server()
{
  _req_set_angular_target_service_server = create_service<robotem_rovne::srv::AngularTarget>(
    "cmd_robot/set_angular_target",
    [this](robotem_rovne::srv::AngularTarget::Request::SharedPtr request,
           robotem_rovne::srv::AngularTarget::Response::SharedPtr /* response */)
    {
      RCLCPP_INFO(get_logger(), "set angular target request received: %0.2f", request->target_angle_rad);
    });
}

void Node::init_ctrl_loop()
{
  _ctrl_loop_timer = create_wall_timer(CTRL_LOOP_RATE, [this]() { this->ctrl_loop(); });
}

void Node::ctrl_loop()
{
  /* TODO */
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* t07 */
