/**
 * Copyright (c) 2024 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/robotem_rovne/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <robotem_rovne/Node.h>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

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
, _imu_qos_profile{rclcpp::KeepLast(10), rmw_qos_profile_sensor_data}
{
  init_req_start_service_server();
  init_req_stop_service_server();
  init_req_set_angular_target_service_server();

  init_imu_sub();

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

void Node::init_imu_sub()
{
  auto const imu_topic = std::string("imu");
  auto const imu_topic_deadline = std::chrono::milliseconds(100);
  auto const imu_topic_liveliness_lease_duration = std::chrono::milliseconds(1000);

  _imu_qos_profile.deadline(imu_topic_deadline);
  _imu_qos_profile.liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC);
  _imu_qos_profile.liveliness_lease_duration(imu_topic_liveliness_lease_duration);

  _imu_sub_options.event_callbacks.deadline_callback =
    [this, imu_topic](rclcpp::QOSDeadlineRequestedInfo & event) -> void
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5*1000UL,
                            "deadline missed for \"%s\" (total_count: %d, total_count_change: %d).",
                            imu_topic.c_str(), event.total_count, event.total_count_change);
    };

  _imu_sub_options.event_callbacks.liveliness_callback =
    [this, imu_topic](rclcpp::QOSLivelinessChangedInfo & event) -> void
    {
      if (event.alive_count > 0)
      {
        RCLCPP_INFO(get_logger(), "liveliness gained for \"%s\"", imu_topic.c_str());
      }
      else
      {
        RCLCPP_WARN(get_logger(), "liveliness lost for \"%s\"", imu_topic.c_str());
      }
    };

  _imu_sub = create_subscription<sensor_msgs::msg::Imu>(
    imu_topic,
    _imu_qos_profile,
    [this](sensor_msgs::msg::Imu::SharedPtr const msg)
    {
      _yaw_actual = static_cast<double>(tf2::getYaw(msg->orientation)) * rad;

      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000UL,
                           "IMU Pose (theta) | (x,y,z,w): %0.2f | %0.2f %0.2f %0.2f %0.2f",
                           _yaw_actual.numerical_value_in(deg),
                           msg->orientation.x,
                           msg->orientation.y,
                           msg->orientation.z,
                           msg->orientation.w);
    },
    _imu_sub_options);
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
