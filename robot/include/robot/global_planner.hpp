#pragma once

#include "robot/gps_coordinate.hpp"
#include "robot/rover_state.hpp"
#include <stdexcept>
#include <sensor_msgs/msg/imu.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>


class GlobalPlanner : public rclcpp::Node
{
public:
  GlobalPlanner(double goal_lat, double goal_lon);

private:
  RoverState    state_;        
  GPSCoordinate goal_;         

  bool gps_received_;          // dont command until 1st Gps data is recieved
  bool imu_received_;          // don't command until first IMU data is recieved

  static constexpr double GOAL_TOLERANCE_M  = 0.5;   
  static constexpr double LINEAR_SPEED_MPS  = 0.8;   
  static constexpr double KP_ANGULAR        = 0.05; //turning strength 
  static constexpr double MAX_ANGULAR_RADS  = 0.8;   //max turnign speed

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr       imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr     odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr      cmd_pub_;
  rclcpp::TimerBase::SharedPtr                                 control_timer_;

  void gpsCallback       (const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void imuCallback       (const sensor_msgs::msg::Imu::SharedPtr msg);
  void odomSpeedCallback (const nav_msgs::msg::Odometry::SharedPtr msg);
  void controlLoop       ();  

  static double normaliseAngle(double angle_deg);

  static double clamp(double val, double max_abs);
};