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

  raw_imu_subscriber_ = this->create_subscription<novatel_oem7_msgs::msg::RAWIMU>(
      "/novatel_top/rawimu", qos_profile,
      std::bind(&TakeHome::jitter_callback, this, std::placeholders::_1));

  metric_publisher_ = this->create_publisher<std_msgs::msg::Float32>("metrics_output", qos_profile);

  slip_publisher_rr_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/rr", qos_profile);
  slip_publisher_rl_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/rl", qos_profile);
  slip_publisher_fr_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/fr", qos_profile);
  slip_publisher_fl_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/fl", qos_profile);

  jitter_publisher_ = this->create_publisher<std_msgs::msg::Float32>("imu_top/jitter", qos_profile);
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

  const float w_r_ = 1.523;
  const float w_f_ = 1.638;
  const float l_f_ = 1.7238;
  current_velocity_ = odom_msg->twist.twist.linear.x;
  float current_lateral_velocity_ = odom_msg->twist.twist.linear.y;
  float yaw_rate = odom_msg->twist.twist.angular.z;

  // rr
  float v_x_rr = current_velocity_ - 0.5 * yaw_rate * w_r_;
  double slip_ratio_rr = (rear_right_speed_ - v_x_rr) / v_x_rr;

  std_msgs::msg::Float32 slip_msg_rr;
  slip_msg_rr.data = slip_ratio_rr;
  slip_publisher_rr_->publish(slip_msg_rr);

  // rl
  float v_x_rl = current_velocity_ + 0.5 * yaw_rate * w_r_;
  double slip_ratio_rl = (rear_left_speed_ - v_x_rl) / v_x_rl;

  std_msgs::msg::Float32 slip_msg_rl;
  slip_msg_rl.data = slip_ratio_rl;
  slip_publisher_rl_->publish(slip_msg_rl);

  // fr
  float v_x_fr = current_velocity_ - 0.5 * yaw_rate * w_f_;
  float v_y_fr = current_lateral_velocity_ + yaw_rate * l_f_;
  float v_delta_x_fr = cos(steering_angle_rad_) * v_x_fr - sin(steering_angle_rad_) * v_y_fr;
  double slip_ratio_fr = (front_right_speed_ - v_delta_x_fr) / v_delta_x_fr;

  std_msgs::msg::Float32 slip_msg_fr;
  slip_msg_fr.data = slip_ratio_fr;
  slip_publisher_fr_->publish(slip_msg_fr);

  // fl
  float v_x_fl = current_velocity_ + 0.5 * yaw_rate * w_f_;
  float v_y_fl = current_lateral_velocity_ + yaw_rate * l_f_;
  float v_delta_x_fl = cos(steering_angle_rad_) * v_x_fl - sin(steering_angle_rad_) * v_y_fl;
  double slip_ratio_fl = (front_left_speed_ - v_delta_x_fl) / v_delta_x_fl;

  std_msgs::msg::Float32 slip_msg_fl;
  slip_msg_fl.data = slip_ratio_fl;
  slip_publisher_fl_->publish(slip_msg_fl);
}

void TakeHome::wheel_speed_callback(
    raptor_dbw_msgs::msg::WheelSpeedReport::ConstSharedPtr wheel_msg)
{
  rear_right_speed_ = wheel_msg->rear_right * 0.277778;   // kmph to m/s
  rear_left_speed_ = wheel_msg->rear_left * 0.277778;     // kmph to m/s
  front_right_speed_ = wheel_msg->front_right * 0.277778; // kmph to m/s
  front_left_speed_ = wheel_msg->front_left * 0.277778;   // kmph to m/s
}

void TakeHome::steering_callback(
    raptor_dbw_msgs::msg::SteeringExtendedReport::ConstSharedPtr steering_msg)
{
  steering_angle_rad_ = steering_msg->primary_steering_angle_fbk * M_PI / 180.0; // deg to rads
}

void TakeHome::jitter_callback(
    novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr imu_msg)
{
  rclcpp::Time current_time = imu_msg->header.stamp;
  imu_timestamps_.push_back(current_time);

  while (!imu_timestamps_.empty() &&
         (current_time - imu_timestamps_.front()).seconds() > 1.0)
  {
    imu_timestamps_.pop_front();
  }

  std::vector<double> deltas;
  for (size_t i = 1; i < imu_timestamps_.size(); ++i)
  {
    double delta = (imu_timestamps_[i] - imu_timestamps_[i - 1]).seconds();
    deltas.push_back(delta);
  }

  double variance = 0.0;
  if (deltas.size() >= 2)
  {
    double mean = std::accumulate(deltas.begin(), deltas.end(), 0.0) / deltas.size();
    for (double d : deltas)
    {
      variance += (d - mean) * (d - mean);
    }
    variance /= (deltas.size() - 1);
  }

  std_msgs::msg::Float32 jitter_msg;
  jitter_msg.data = variance;
  jitter_publisher_->publish(jitter_msg);
}

RCLCPP_COMPONENTS_REGISTER_NODE(TakeHome)
