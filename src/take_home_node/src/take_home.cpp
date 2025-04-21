#include "take_home_node/take_home.hpp"
#include <rclcpp_components/register_node_macro.hpp>

TakeHome::TakeHome(const rclcpp::NodeOptions &options)
    : Node("take_home_metrics", options)
{

  auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

  // Look at the hpp file to define all class variables, including subscribers
  // A subscriber will "listen" to a topic and whenever a message is published to it, the subscriber
  // will pass it onto the attached callback (`TakeHome::odometry_callback` in this instance)
  odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "vehicle/uva_odometry", qos_profile,
      std::bind(&TakeHome::odometry_callback, this, std::placeholders::_1));

  // Wheel speed subscriber
  wheel_speed_subscriber_ = this->create_subscription<raptor_dbw_msgs::msg::WheelSpeedReport>(
      "/raptor_dbw_interface/wheel_speed_report", qos_profile,
      std::bind(&TakeHome::wheel_speed_callback, this, std::placeholders::_1));

  // Steering angle subscriber
  steering_subscriber_ = this->create_subscription<raptor_dbw_msgs::msg::SteeringExtendedReport>(
      "/raptor_dbw_interface/steering_extended_report", qos_profile,
      std::bind(&TakeHome::steering_callback, this, std::placeholders::_1));


  metric_publisher_ = this->create_publisher<std_msgs::msg::Float32>("metrics_output", qos_profile);
  slip_publisher_rr_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/rr", qos_profile);
}

//
/**
 * Whenever a message is published to the topic "vehicle/uva_odometry" the subscriber passes the message onto this callback
 * To see what is in each message look for the corresponding .msg file in this repository
 * For instance, when running `ros2 bag info` on the given bag, we see the wheel speed report has message type of raptor_dbw_msgs/msgs/WheelSpeedReport
 * and from the corresponding WheelSpeedReport.msg file we see that we can do msg->front_left for the front left speed for instance.
 * For the built in ROS2 messages, we can find the documentation online: e.g. https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html
 */
void TakeHome::odometry_callback(nav_msgs::msg::Odometry::ConstSharedPtr odom_msg)
{
  float position_x = odom_msg->pose.pose.position.x;
  float position_y = odom_msg->pose.pose.position.y;
  float position_z = odom_msg->pose.pose.position.z;

  // Do stuff with this callback! or more, idc
  std_msgs::msg::Float32 metric_msg;
  metric_msg.data = (position_x + position_y + position_z) / (position_x + position_z); // Example metric calculation
  metric_publisher_->publish(metric_msg);

  float current_velocity_ = odom_msg->twist.twist.linear.x;
  double slip_ratio_rr = (rear_right_speed_ - current_velocity_) / current_velocity_;
  
  std_msgs::msg::Float32 slip_msg;
  slip_msg.data = slip_ratio_rr;
  slip_publisher_rr_->publish(slip_msg);
}



void TakeHome::wheel_speed_callback(
    raptor_dbw_msgs::msg::WheelSpeedReport::ConstSharedPtr wheel_msg)
{
  rear_right_speed_ = wheel_msg->rear_right * 0.277778; // kmph to m/s
  // TODO: Repeat for rear_left, front_left, front_right
}

void TakeHome::steering_callback(
    raptor_dbw_msgs::msg::SteeringExtendedReport::ConstSharedPtr steering_msg)
{
  steering_angle_rad_ = steering_msg->primary_steering_angle_fbk * M_PI / 180.0; // deg to rads
}

RCLCPP_COMPONENTS_REGISTER_NODE(TakeHome)
