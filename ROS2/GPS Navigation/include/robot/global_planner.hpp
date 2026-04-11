#pragma once

#include "robot/gps_coordinate.hpp"
#include "robot/rover_state.hpp"

#include <sensor_msgs/msg/imu.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>

// GlobalPlanner IS-A ROS2 node (inherits from rclcpp::Node).
// It subscribes to GPS, IMU, and odometry, and publishes cmd_vel
// at a fixed rate to drive the rover toward the goal coordinate.

class GlobalPlanner : public rclcpp::Node
{
public:
  // Constructor takes the goal coordinate as arguments
  GlobalPlanner(double goal_lat, double goal_lon);

private:
  // ── Data members ──────────────────────────────────────────
  RoverState    state_;        // current pose (updated by callbacks)
  GPSCoordinate goal_;         // target GPS coordinate

  bool gps_received_;          // guard: don't command until first GPS fix
  bool imu_received_;          // guard: don't command until first IMU data

  // Tuning parameters
  static constexpr double GOAL_TOLERANCE_M  = 1.5;   // stop within 1.5 m
  static constexpr double LINEAR_SPEED_MPS  = 0.8;   // forward speed
  static constexpr double KP_ANGULAR        = 0.03;  // P-gain for heading
  static constexpr double MAX_ANGULAR_RADS  = 0.8;   // rad/s clamp

  // ── ROS interfaces ────────────────────────────────────────
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr       imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr     odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr      cmd_pub_;
  rclcpp::TimerBase::SharedPtr                                 control_timer_;

  // ── Private methods ───────────────────────────────────────
  void gpsCallback       (const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void imuCallback       (const sensor_msgs::msg::Imu::SharedPtr msg);
  void odomSpeedCallback (const nav_msgs::msg::Odometry::SharedPtr msg);
  void controlLoop       ();  // called at 10 Hz by the timer

  // Normalise any angle to [-180, +180] degrees
  static double normaliseAngle(double angle_deg);

  // Clamp a value between -max and +max
  static double clamp(double val, double max_abs);
};