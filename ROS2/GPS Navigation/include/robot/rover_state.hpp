#pragma once
#include "robot/gps_coordinate.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <sensor_msgs/msg/imu.hpp>


// Holds everything we know about the rover's current state.
// Updated by two separate ROS callbacks: GPS and odometry.
class RoverState
{
public:
  GPSCoordinate position;   // latest GPS fix
  double heading_deg = 0.0; // yaw in degrees, ROS convention:
                            // 0=East, 90=North (converted below)
  double speed_mps  = 0.0;  // forward speed from odometry

  // Called whenever a new /gps/fix message arrives
  void updateFromGPS(const sensor_msgs::msg::NavSatFix & msg)
  {
    position.latitude  = msg.latitude;
    position.longitude = msg.longitude;
  }

  // Called whenever a new /odom message arrives.
  // Extracts yaw from the quaternion and forward speed from twist.
  void updateFromIMU(const sensor_msgs::msg::Imu & msg)
{
  tf2::Quaternion q(
    msg.orientation.x,
    msg.orientation.y,
    msg.orientation.z,
    msg.orientation.w
  );
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  // ENU yaw (rad, CCW from East) → compass bearing (deg, CW from North)
  double compass = 90.0 - (yaw * (180.0 / M_PI));

  // Normalise to [0, 360)
  heading_deg = std::fmod(compass + 360.0, 360.0);
}

void updateSpeedFromOdom(const nav_msgs::msg::Odometry & msg)
{
  speed_mps = msg.twist.twist.linear.x;
}
};