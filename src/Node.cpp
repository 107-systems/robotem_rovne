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
, _linear_vel{0. * m/s}
, _angular_vel{0. * rad/s}
, _imu_qos_profile{rclcpp::KeepLast(10), rmw_qos_profile_sensor_data}
, _motor_left_qos_profile{rclcpp::KeepLast(1), rmw_qos_profile_sensor_data}
, _motor_right_qos_profile{rclcpp::KeepLast(1), rmw_qos_profile_sensor_data}
, _robot_state{State::Stopped}
, _motor_base_vel{0. * m/s}
{
  init_req_start_service_server();
  init_req_stop_service_server();
  init_req_set_angular_target_service_server();

  init_cmd_vel_sub();
  init_imu_sub();

  init_motor_left_pub();
  init_motor_right_pub();

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
      if (_robot_state == State::Stopped)
      {
        _robot_state = State::Starting;
      }
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
      if (_robot_state != State::Stopped)
      {
        _robot_state = State::Stopping;
      }
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
      _yaw_target = request->target_angle_rad * rad;
    });
}

void Node::init_cmd_vel_sub()
{
  _cmd_vel_sub = create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel",
    1,
    [this](geometry_msgs::msg::Twist::SharedPtr const msg)
    {
      _linear_vel = static_cast<double>(msg->linear.x) * m/s;
      _angular_vel = static_cast<double>(msg->angular.z) * rad/s;

      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000UL,
                           "linear_vel = %0.2f m/s, angular_vel = %0.2f",
                           _linear_vel.numerical_value_in(m/s),
                           _angular_vel.numerical_value_in(deg/s));
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

void Node::init_motor_left_pub()
{
  auto const motor_left_topic = std::string("/motor/left/target");
  auto const motor_left_topic_deadline = std::chrono::milliseconds(100);
  auto const motor_left_topic_liveliness_lease_duration = std::chrono::milliseconds(1000);

  _motor_left_qos_profile.deadline(motor_left_topic_deadline);
  _motor_left_qos_profile.liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC);
  _motor_left_qos_profile.liveliness_lease_duration(motor_left_topic_liveliness_lease_duration);

  _motor_left_pub = create_publisher<std_msgs::msg::Float32>(motor_left_topic, _motor_left_qos_profile);
}

void Node::pub_motor_left(quantity<m/s> const velocity)
{
  std_msgs::msg::Float32 msg;
  msg.data = velocity.numerical_value_in(m/s);
  _motor_left_pub->publish(msg);
}

void Node::init_motor_right_pub()
{
  auto const motor_right_topic = std::string("/motor/right/target");
  auto const motor_right_topic_deadline = std::chrono::milliseconds(100);
  auto const motor_right_topic_liveliness_lease_duration = std::chrono::milliseconds(1000);

  _motor_right_qos_profile.deadline(motor_right_topic_deadline);
  _motor_right_qos_profile.liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC);
  _motor_right_qos_profile.liveliness_lease_duration(motor_right_topic_liveliness_lease_duration);

  _motor_right_pub = create_publisher<std_msgs::msg::Float32>(motor_right_topic, _motor_right_qos_profile);
}

void Node::pub_motor_right(quantity<m/s> const velocity)
{
  std_msgs::msg::Float32 msg;
  msg.data = velocity.numerical_value_in(m/s);
  _motor_right_pub->publish(msg);
}

void Node::init_ctrl_loop()
{
  _ctrl_loop_timer = create_wall_timer(CTRL_LOOP_RATE, [this]() { this->ctrl_loop(); });
}

void Node::ctrl_loop()
{
  switch (_robot_state)
  {
    case State::Stopped:  handle_Stopped(); break;
    case State::Orienting: handle_Orienting(); break;
    case State::Starting: handle_Starting(); break;
    case State::Driving:  handle_Driving(); break;
    case State::Stopping: handle_Stopping(); break;
  }
}

void Node::control_yaw()
{
  auto const yaw_err = (_yaw_target - _yaw_actual);

  double const k = 0.01;
  double const pid_res = k * yaw_err.numerical_value_in(deg);

  auto const motor_vel_lower_limit = _motor_base_vel - 0.2 * m/s;
  auto const motor_vel_upper_limit = _motor_base_vel + 0.2 * m/s;

  auto motor_left_vel  = _motor_base_vel + pid_res * m/s;
  motor_left_vel = std::max(motor_left_vel, motor_vel_lower_limit);
  motor_left_vel = std::min(motor_left_vel, motor_vel_upper_limit);

  auto motor_right_vel = _motor_base_vel - pid_res * m/s;
  motor_right_vel = std::max(motor_right_vel, motor_vel_lower_limit);
  motor_right_vel = std::min(motor_right_vel, motor_vel_upper_limit);

  RCLCPP_INFO(get_logger(),
              "actual = %0.2f, target = %0.2f, error = %0.2f, pid_res = %0.2f, LEFT = %0.2f m/s, RIGHT = %0.2f m/s",
              _yaw_actual.numerical_value_in(deg),
              _yaw_target.numerical_value_in(deg),
              yaw_err.numerical_value_in(deg),
              pid_res,
              motor_left_vel.numerical_value_in(m/s),
              motor_right_vel.numerical_value_in(m/s));

  pub_motor_left (motor_left_vel);
  pub_motor_right(motor_right_vel);
}

void Node::handle_Stopped()
{
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000UL, "State::Stopped");

  _motor_base_vel = 0. * m/s;

  pub_motor_left (0. * m/s);
  pub_motor_right(0. * m/s);
}

void Node::handle_Orienting()
{
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000UL, "State::Orienting");

  _motor_base_vel = 0. * m/s;

  control_yaw();

  auto const yaw_err = (_yaw_target - _yaw_actual);
  if ( yaw_err < (-5. * deg).in(rad) && yaw_err < (5. * deg).in(rad))
    _robot_state = State::Starting;
}

void Node::handle_Starting()
{
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000UL, "State::Starting");

  control_yaw();

  if (_motor_base_vel < 0.5 * m/s)
  {
    _motor_base_vel += 0.01 * m/s;
  }
  else
  {
    _motor_base_vel  = 0.5 * m/s;
    _robot_state = State::Driving;
  }
}

void Node::handle_Driving()
{
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000UL, "State::Driving");

  control_yaw();
}

void Node::handle_Stopping()
{
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000UL, "State::Stopping");

  control_yaw();

  if (_motor_base_vel > 0. * m/s)
  {
    _motor_base_vel -= 0.025 * m/s;
  }
  else
  {
    _motor_base_vel  = 0. * m/s;
    _robot_state = State::Stopped;
  }
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* t07 */
