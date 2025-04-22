#pragma once

// Here we include message types which we can subscribe to or publish
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/odometry.hpp> 

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cmath>

#include <raptor_dbw_msgs/msg/wheel_speed_report.hpp>
#include <raptor_dbw_msgs/msg/steering_extended_report.hpp>

#include <novatel_oem7_msgs/msg/rawimu.hpp>
#include <deque>

class TakeHome : public rclcpp::Node {
 public:
  TakeHome(const rclcpp::NodeOptions& options);

  void odometry_callback(nav_msgs::msg::Odometry::ConstSharedPtr odom_msg);
  void wheel_speed_callback(raptor_dbw_msgs::msg::WheelSpeedReport::ConstSharedPtr odom_msg);
  void steering_callback(raptor_dbw_msgs::msg::SteeringExtendedReport::ConstSharedPtr odom_msg);
  void jitter_callback(novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr imu_msg);

 private:

  // Subscribers and Publishers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr metric_publisher_;
  
  rclcpp::Subscription<raptor_dbw_msgs::msg::WheelSpeedReport>::SharedPtr wheel_speed_subscriber_;
  rclcpp::Subscription<raptor_dbw_msgs::msg::SteeringExtendedReport>::SharedPtr steering_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_publisher_rr_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_publisher_rl_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_publisher_fr_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_publisher_fl_;
  
  rclcpp::Subscription<novatel_oem7_msgs::msg::RAWIMU>::SharedPtr raw_imu_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr jitter_publisher_;

  // variables to store data
  double current_velocity_; // (m/s)
  double rear_right_speed_; // (m/s)
  double rear_left_speed_; // (m/s)
  double front_right_speed_; // (m/s)
  double front_left_speed_; // (m/s)
  double steering_angle_rad_; // (rad)
  std::deque<rclcpp::Time> imu_timestamps_;

};
