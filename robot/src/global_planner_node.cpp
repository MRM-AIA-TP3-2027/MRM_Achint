#include "robot/global_planner.hpp"



GlobalPlanner::GlobalPlanner(double goal_lat, double goal_lon)
: rclcpp::Node("global_planner"),
  goal_(goal_lat, goal_lon),
  gps_received_(false),
  imu_received_(false)
{
  gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
    "/gps/fix", 10,
    [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
      gpsCallback(msg);
    }
  );

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/imu/data", 10,
    [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
      imuCallback(msg);
    }
  );

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10,
    [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
      odomSpeedCallback(msg);
    }
  );

  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  control_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),
    [this]() { controlLoop(); }
  );

  RCLCPP_INFO(this->get_logger(),
    "GlobalPlanner ready. Goal: (%.6f, %.6f)", goal_lat, goal_lon);
}

void GlobalPlanner::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  if (msg->status.status < 0) return;
  state_.updateFromGPS(*msg);
  gps_received_ = true;
}

void GlobalPlanner::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  state_.updateFromIMU(*msg);
  imu_received_ = true;
}

void GlobalPlanner::odomSpeedCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  state_.updateSpeedFromOdom(*msg);
}

void GlobalPlanner::controlLoop()
{
  if (!gps_received_ || !imu_received_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(),
      2000, "Waiting for GPS/IMU fix...");
    return;
  }

  double distance_m   = state_.position.distanceTo(goal_);
  double goal_bearing = state_.position.bearingTo(goal_);

  if (distance_m < GOAL_TOLERANCE_M) {
    RCLCPP_INFO(this->get_logger(),
      "Goal reached! Distance: %.2f m. Stopping.", distance_m);
    cmd_pub_->publish(geometry_msgs::msg::Twist());
    control_timer_->cancel();
    return;
  }

  double heading_error = normaliseAngle(goal_bearing - state_.heading_deg + 180.0);

double angular_z = clamp(-KP_ANGULAR * heading_error, MAX_ANGULAR_RADS);

double dist_scale = std::min(1.0, distance_m / 3.0);
double alignment  = std::cos(heading_error * M_PI / 180.0);
double linear_x   = LINEAR_SPEED_MPS * std::max(0.0, alignment) * dist_scale;

if (std::abs(heading_error) > 10.0) linear_x = 0.0;  // or 3.0

// Scale speed by distance — slow down when close
// double dist_scale   = std::min(1.0, distance_m / 3.0);  // full speed beyond 3m, scales down inside
// double alignment    = std::cos(heading_error * M_PI / 180.0);
// double linear_x     = LINEAR_SPEED_MPS * std::max(0.0, alignment) * dist_scale;

// if (std::abs(heading_error) >5) linear_x = 0.0;
//   double heading_error = normaliseAngle(goal_bearing - state_.heading_deg+180); // removed +180

// double angular_z = clamp(-KP_ANGULAR * heading_error, MAX_ANGULAR_RADS);

// double alignment = std::cos(heading_error * M_PI / 180.0);
// double linear_x  = LINEAR_SPEED_MPS * std::max(0.0, alignment);

// if (std::abs(heading_error) > 90.0) linear_x = 0.0;
// double heading_error = normaliseAngle(goal_bearing - state_.heading_deg+180); // FIXED

// double angular_z = 0.0;
// double linear_x  = 0.0;

// if (std::abs(heading_error) > 20.0) {
//     // Still turning — don't move forward yet
//     angular_z = clamp(-KP_ANGULAR * heading_error, MAX_ANGULAR_RADS);
// } else {
//     // Aligned — drive straight
//     linear_x = LINEAR_SPEED_MPS;
// }


 
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x  = linear_x;
  cmd.angular.z = angular_z;
  cmd_pub_->publish(cmd);

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
    "Dist: %.1f m | GoalBearing: %.1f° | Heading: %.1f° | Error: %.1f° | "
    "lin=%.2f ang=%.2f",
    distance_m, goal_bearing, state_.heading_deg,
    heading_error, linear_x, angular_z);
}

double GlobalPlanner::normaliseAngle(double angle_deg)
{
  while (angle_deg >  180.0) angle_deg -= 360.0;
  while (angle_deg < -180.0) angle_deg += 360.0;
  return angle_deg;
}

double GlobalPlanner::clamp(double val, double max_abs)
{
  if (val >  max_abs) return  max_abs;
  if (val < -max_abs) return -max_abs;
  return val;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  double goal_lat = 0.0, goal_lon = 0.0;
  if (argc == 3) {
    goal_lat = std::stod(argv[1]);
    goal_lon = std::stod(argv[2]);
  }

  auto node = std::make_shared<GlobalPlanner>(goal_lat, goal_lon);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}