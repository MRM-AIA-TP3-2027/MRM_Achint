// #include <new>
// #include "rclcpp/rclcpp.hpp"
// #include "robot/global_planner.hpp"
// #include "robot/rover_state.hpp"
// #include "robot/obstacle_avoidance.hpp"

// #include <sensor_msgs/msg/nav_sat_fix.hpp>
// #include <sensor_msgs/msg/imu.hpp>
// #include <nav_msgs/msg/odometry.hpp>
// #include <sensor_msgs/msg/laser_scan.hpp>
// #include <sensor_msgs/msg/image.hpp>
// #include <geometry_msgs/msg/twist.hpp>
// #include <cmath>
// #include <limits>
// #include <numeric>
// #include <vector>
// #include <chrono>


// //  GLOBAL PLANNER

// GlobalPlanner::GlobalPlanner(double goal_lat, double goal_lon)
// : goal_(goal_lat, goal_lon)
// {
// }

// geometry_msgs::msg::Twist
// GlobalPlanner::computeCommand(const RoverState& state)
// {
//     geometry_msgs::msg::Twist cmd;

//     double distance_m   = state.position.distanceTo(goal_);
//     double goal_bearing = state.position.bearingTo(goal_);

//     if (distance_m < GOAL_TOLERANCE_M || goal_reached_) {
//         if (!goal_reached_) {
//             RCLCPP_INFO(rclcpp::get_logger("global_planner"),
//                 "[GOAL] Reached! dist=%.2fm — stopping", distance_m);
//             goal_reached_ = true;
//         }
//         cmd.linear.x  = 0.0;
//         cmd.angular.z = 0.0;
//         return cmd;
//     }


// double heading_error = normaliseAngle(goal_bearing - state.heading_deg + 180.0);
// double angular_z = clamp(-KP_ANGULAR * heading_error, MAX_ANGULAR_RADS);


// if (std::abs(heading_error) > 90.0) {
//     angular_z = (heading_error > 0) ? -MAX_ANGULAR_RADS : MAX_ANGULAR_RADS;
// }

// double dist_scale = std::min(1.0, distance_m / 3.0);
// double alignment  = std::cos(heading_error * M_PI / 180.0);


// double linear_x;


// if (std::abs(heading_error) > 60.0) {
//     linear_x = 0.0;   
// } else {
//     linear_x = LINEAR_SPEED_MPS * std::max(0.2, alignment) * dist_scale;
// }

// if (linear_x > 0.0)
//     linear_x = std::max(0.05, linear_x);

// cmd.linear.x  = linear_x;
// cmd.angular.z = angular_z;

// return cmd;
// }
// double GlobalPlanner::normaliseAngle(double angle_deg)
// {
//     while (angle_deg >  180.0) angle_deg -= 360.0;
//     while (angle_deg < -180.0) angle_deg += 360.0;
//     return angle_deg;
// }

// double GlobalPlanner::clamp(double val, double max_abs)
// {
//     if (val >  max_abs) return  max_abs;
//     if (val < -max_abs) return -max_abs;
//     return val;
// }


// //  OBSTACLE AVOIDANCE

// void ObstacleAvoidance::processScan(
//     const sensor_msgs::msg::LaserScan::SharedPtr& msg)
// {
//     const int n  = static_cast<int>(msg->ranges.size());
//     angle_min_   = msg->angle_min;
//     angle_inc_   = msg->angle_increment;
//     ranges_      = msg->ranges;

//     auto deg2idx = [&](float deg) -> int {
//         float rad = deg * static_cast<float>(M_PI) / 180.0f;
//         int   idx = static_cast<int>(std::round((rad - angle_min_) / angle_inc_));
//         return std::max(0, std::min(n - 1, idx));
//     };

//     //  Front cone minimum
//     int lo = deg2idx(-FRONT_FOV_DEG);
//     int hi = deg2idx( FRONT_FOV_DEG);
//     if (lo > hi) std::swap(lo, hi);

//     front_min_       = 999.f;
//     front_min_angle_ = 0.0f;
//     for (int i = lo; i <= hi; ++i) {
//         float r = ranges_[i];
//         if (std::isfinite(r) && r > 0.05f && r < front_min_) {
//             front_min_       = r;
//             front_min_angle_ = idx2deg(i);
//         }
//     }

//     //  Left/right side averages
//     auto sideAvg = [&](float from_deg, float to_deg) -> float {
//         int a = deg2idx(from_deg);
//         int b = deg2idx(to_deg);
//         if (a > b) std::swap(a, b);
//         float sum = 0.f; int cnt = 0;
//         for (int i = a; i <= b; ++i) {
//             float r = ranges_[i];
//             if (std::isfinite(r) && r > 0.05f) { sum += r; cnt++; }
//         }
//         return (cnt > 0) ? sum / cnt : 999.f;
//     };

//     left_avg_       = sideAvg( 10.f,  90.f);
//     right_avg_      = sideAvg(-90.f, -10.f);
//     back_left_avg_  = sideAvg( 90.f, 170.f);
//     back_right_avg_ = sideAvg(-170.f, -90.f);
//     rear_min_       = sideAvg(150.f, 180.f);
//     rear_min_       = std::min(rear_min_, sideAvg(-180.f, -150.f));

//     //  Find best gap direction
//     {
//         const float SECTOR_DEG  = 15.0f;
//         float best_gap   = goal_bearing_;
//         float best_score = 1e9f;
//         const float OPEN_THRESH = 2.0f;
//         const float SEARCH_FOV  = 180.0f;

//         // Decrement suppression counter once per scan
//         if (suppress_goal_pull_) {
//             if (--suppress_scan_count_ <= 0) suppress_goal_pull_ = false;
//         }

//         for (float centre = -SEARCH_FOV; centre <= SEARCH_FOV; centre += SECTOR_DEG) {
//             int a2 = deg2idx(centre - SECTOR_DEG * 0.5f);
//             int b2 = deg2idx(centre + SECTOR_DEG * 0.5f);
//             if (a2 > b2) std::swap(a2, b2);
//             float sum2 = 0.f; int cnt2 = 0;
//             for (int i = a2; i <= b2; ++i) {
//                 float r = ranges_[i];
//                 if (std::isfinite(r) && r > 0.05f) { sum2 += r; cnt2++; }
//             }
//             float avg_clear = (cnt2 > 0) ? sum2 / cnt2 : 999.f;
//             if (avg_clear < OPEN_THRESH) continue;

//             float diff = std::abs(centre - goal_bearing_);
//             if (diff > 180.f) diff = 360.f - diff;
// float goal_penalty = suppress_goal_pull_ ? (diff / 120.0f) : (diff / 45.0f);
//           float score = goal_penalty - (avg_clear * 0.8f);
//             if (score < best_score) {
//                 best_score = score;
//                 best_gap   = centre;
//             }
//         }
//        gap_angle_ = best_gap;



//         // Measure width of the chosen gap
//         float width = 0.0f;
//         for (float c = best_gap - SEARCH_FOV; c <= best_gap + SEARCH_FOV; c += SECTOR_DEG) {
//             int a2 = deg2idx(c - SECTOR_DEG * 0.5f);
//             int b2 = deg2idx(c + SECTOR_DEG * 0.5f);
//             if (a2 > b2) std::swap(a2, b2);
//             float sum2 = 0.f; int cnt2 = 0;
//             for (int i = a2; i <= b2; ++i) {
//                 float r = ranges_[i];
//                 if (std::isfinite(r) && r > 0.05f) { sum2 += r; cnt2++; }
//             }
//             float avg_clear = (cnt2 > 0) ? sum2 / cnt2 : 999.f;
//             if (avg_clear > ESCAPE_CLEAR_THRESH)
//                 width += SECTOR_DEG;
//         }
//         gap_width_deg_ = width;
//     }

//     if (!logged_once_) {
//         RCLCPP_INFO(rclcpp::get_logger("obstacle_avoidance"),
//             "[SCAN] n=%d angle_min=%.2f inc=%.4f front_fov=±%.0f° front=%.2f L=%.2f R=%.2f",
//             n, angle_min_, angle_inc_, FRONT_FOV_DEG,
//             front_min_, left_avg_, right_avg_);
//         logged_once_ = true;
//     }

//     updateState();
// }

// void ObstacleAvoidance::processDepth(
//     const sensor_msgs::msg::Image::SharedPtr& msg)
// {
//     if (msg->encoding != "32FC1") return;

//     const float* data = reinterpret_cast<const float*>(msg->data.data());
//     const int W = static_cast<int>(msg->width);
//     const int H = static_cast<int>(msg->height);

//     int col0 = W * 4 / 10, col1 = W * 6 / 10;
//     int row0 = H * 2 / 10, row1 = H * 8 / 10;

//     float min_depth = 999.f;
//     for (int r = row0; r < row1; ++r)
//         for (int c = col0; c < col1; ++c) {
//             float d = data[r * W + c];
//             if (std::isfinite(d) && d > 0.05f)
//                 min_depth = std::min(min_depth, d);
//         }

//     if (min_depth < front_min_) {
//         RCLCPP_INFO(rclcpp::get_logger("obstacle_avoidance"),
//             "[DEPTH] stereo=%.2f overrides lidar=%.2f", min_depth, front_min_);
//         front_min_ = min_depth;
//         updateState();
//     }
// }

// //  updateState
// void ObstacleAvoidance::updateState()
// {
//     using SC = std::chrono::steady_clock;
//     const auto now = SC::now();

//     bool obstacle_ahead;
//     if (state_ == AvoidState::NAVIGATING)
//         obstacle_ahead = (front_min_ < CAUTION_DIST);
//     else
//         obstacle_ahead = (front_min_ < CLEAR_DIST);   // hysteresis

//     //  AVOIDING → clear
//     if (!obstacle_ahead && state_ == AvoidState::AVOIDING) {
//         clear_count_++;
//         if (clear_count_ >= 2) {
//             RCLCPP_INFO(rclcpp::get_logger("obstacle_avoidance"),
//                 "[OBS] Path clear — NAVIGATING (front=%.2f)", front_min_);
//             state_ = AvoidState::NAVIGATING;
//             best_front_while_avoiding_ = 999.f;
//             clear_count_ = 0;
//             full_escape_attempts_ = 0;
//         }
//         return;
//     } else {
//         clear_count_ = 0;
//     }

//     //  NAVIGATING → obstacle
//     if (obstacle_ahead && state_ == AvoidState::NAVIGATING) {
//         state_                     = AvoidState::AVOIDING;
//         avoiding_since_            = now;
//         best_front_while_avoiding_ = front_min_;
//         turn_right_ = ((right_avg_ + back_right_avg_) > (left_avg_ + back_left_avg_));
//           float dir = turn_right_ ? -1.0f : 1.0f;
//     if (dir * last_avoid_direction_ > 0) {
//         same_dir_count_++;
//         if (same_dir_count_ > 2) {
//             turn_right_ = !turn_right_;
//             same_dir_count_ = 0;
//             RCLCPP_WARN(rclcpp::get_logger("obstacle_avoidance"),
//                 "[OBS] Same direction 3x — flipping turn");
//         }
//     } else {
//         same_dir_count_ = 0;
//     }
//     last_avoid_direction_ = dir;

//     suppress_goal_pull_  = true;
//     suppress_scan_count_ = 8;
      
//         RCLCPP_WARN(rclcpp::get_logger("obstacle_avoidance"),
//             "[OBS] OBSTACLE front=%.2f L=%.2f R=%.2f — turning %s",
//             front_min_, left_avg_, right_avg_,
//             turn_right_ ? "RIGHT" : "LEFT");
//         return;
//     }

//     //  AVOIDING — check for stuck / local minima
//     if (state_ == AvoidState::AVOIDING) {
//         if (front_min_ > best_front_while_avoiding_ + STUCK_IMPROVE_M) {
//             best_front_while_avoiding_ = front_min_;
//             avoiding_since_            = now;
//         }

//        double elapsed = std::chrono::duration<double>(now - avoiding_since_).count();

// if (elapsed > STUCK_TIMEOUT_S) {
//     state_        = AvoidState::ESCAPING;
//     escape_start_ = now;
//     escape_phase_ = 0;
//     clear_count_  = 0;

//     full_escape_attempts_++;

//     //  flip direction after first failure
//     if (full_escape_attempts_ >= 1) {
//         turn_right_ = !turn_right_;
//         suppress_goal_pull_ = false;

//         RCLCPP_ERROR(rclcpp::get_logger("obstacle_avoidance"),
//             "[STRATEGY] Switching side — now turning %s",
//             turn_right_ ? "RIGHT" : "LEFT");
//     }

//     //  force goal after repeated failures
//     if (full_escape_attempts_ >= 2) {
//         suppress_goal_pull_ = false;
//         gap_angle_ = goal_bearing_;

//         RCLCPP_ERROR(rclcpp::get_logger("obstacle_avoidance"),
//             "[STRATEGY] Forcing GOAL direction");
//     }

//     RCLCPP_ERROR(rclcpp::get_logger("obstacle_avoidance"),
//         "[OBS] LOCAL MINIMA — stuck %.1fs, best_front=%.2f → ESCAPING",
//         elapsed, best_front_while_avoiding_);
// }
//     }
// }

// //  computeAvoidance
// geometry_msgs::msg::Twist ObstacleAvoidance::computeAvoidance()
// {
//     auto clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
//     using SC   = std::chrono::steady_clock;
//     geometry_msgs::msg::Twist cmd;

//     // ── ESCAPING phase ──────────────────────────────────────────────────────
//     if (state_ == AvoidState::ESCAPING) {
//         double elapsed = std::chrono::duration<double>(
//             SC::now() - escape_start_).count();

//         // Phase 0: Reverse 
//         if (escape_phase_ == 0) {
//     float side_diff = right_avg_ - left_avg_;
//     float steer = 0.0f;

//     if (std::abs(side_diff) > 1.0f) {
//         steer = (side_diff < 0) ? +0.3f : -0.3f;
//     }

//     //  REAR SAFETY 
//     if (rear_min_ < 1.0f) {
//         cmd.linear.x  = 0.0;
//         cmd.angular.z = (turn_right_ ? -0.8f : 0.8f);

//         RCLCPP_WARN(rclcpp::get_logger("obstacle_avoidance"),
//             "[ESC] REAR TOO CLOSE — stopping reverse (rear=%.2f)", rear_min_);

//         return cmd;
//     }

//     //  SIDE SAFETY 
//     if (left_avg_ < 0.8f || right_avg_ < 0.8f) {
//         cmd.linear.x  = 0.0;
//         cmd.angular.z = (turn_right_ ? -0.8f : 0.8f);

//         RCLCPP_WARN(rclcpp::get_logger("obstacle_avoidance"),
//             "[ESC] SIDE TOO CLOSE — rotating");

//         return cmd;
//     }

//     // NORMAL REVERSE
//     cmd.linear.x  = ESCAPE_REVERSE_MPS;
//     cmd.angular.z = steer;

//             RCLCPP_WARN_THROTTLE(rclcpp::get_logger("obstacle_avoidance"),
//                 *clock_, 500, "[ESC] BACKING %.1f/%.1fs steer=%.1f L=%.2f R=%.2f",
//                 elapsed, ESCAPE_REVERSE_S, steer, left_avg_, right_avg_);

//             if (elapsed > ESCAPE_REVERSE_S) {
//                 escape_phase_ = 1;
//                 escape_start_ = SC::now();
//             }

//         // Phase 1: Spin until a gap opens
//         } else if (escape_phase_ == 1) {
//             cmd.linear.x  = 0.0;
//             const float spin_sign = turn_right_ ? -1.f : 1.f;
//             cmd.angular.z = spin_sign * HARD_TURN_RAD_S;

//             RCLCPP_WARN_THROTTLE(rclcpp::get_logger("obstacle_avoidance"),
//                 *clock_, 500, "[ESC] SPINNING — front=%.2f need>%.2f",
//                 front_min_, ESCAPE_CLEAR_THRESH);

//             double max_spin   = (3.0 * M_PI) / HARD_TURN_RAD_S;
//             double min_spin_s = M_PI / HARD_TURN_RAD_S;

//             bool rear_clear = (rear_min_ > ESCAPE_CLEAR_THRESH && elapsed > min_spin_s);
//             bool gap_found  = rear_clear ||
//                               (front_min_ > ESCAPE_CLEAR_THRESH &&
//                                gap_width_deg_ >= ESCAPE_GAP_WIDTH_DEG &&
//                                elapsed > min_spin_s);
//             bool spin_timeout = (elapsed > max_spin);

//                 if (gap_found || spin_timeout) {
//                    if (front_min_ < DANGER_DIST) {
      
//     cmd.linear.x  = -0.15f;   
//     cmd.angular.z = turn_right_ ? -0.5f : 0.5f;
//     return cmd;
// }else {
//                         escape_phase_ = 2;
//                         escape_start_ = SC::now();
//                         RCLCPP_WARN(rclcpp::get_logger("obstacle_avoidance"),
//                             "[ESC] Gap found (front=%.2f) — DRIVING", front_min_);
//                     }
//                 }

//         // Phase 2: Drive straight until sides open up
//       } else if (escape_phase_ == 2) {

//     // 🚨 HARD SAFETY STOP
//     if (front_min_ < 0.7f)  {
//         cmd.linear.x  = 0.0;
//         cmd.angular.z = (turn_right_ ? -0.8f : 0.8f);

//         RCLCPP_WARN(rclcpp::get_logger("obstacle_avoidance"),
//             "[ESC] TOO CLOSE — stopping & rotating (front=%.2f)", front_min_);

//         return cmd;
//     }
    

//     // steer toward gap
// float side_diff = left_avg_ - right_avg_;
// float ang = std::clamp(0.2f * side_diff, -HARD_TURN_RAD_S, HARD_TURN_RAD_S);
// ang += (turn_right_ ? -0.3f : 0.3f);
// ang = std::clamp(ang, -HARD_TURN_RAD_S, HARD_TURN_RAD_S);

// cmd.linear.x  = 0.15f;
// cmd.angular.z = ang;
//     RCLCPP_WARN_THROTTLE(rclcpp::get_logger("obstacle_avoidance"),
//         *clock_, 500, "[ESC] DRIVING — gap=%.1f° L=%.2f R=%.2f front=%.2f",
//         gap_angle_, left_avg_, right_avg_, front_min_);

//     // if blocked → restart escape
//     if (front_min_ < DANGER_DIST) {
//     cmd.linear.x  = -0.2f;   // reverse
//     cmd.angular.z = turn_right_ ? -0.6f : 0.6f;

//     RCLCPP_WARN(rclcpp::get_logger("obstacle_avoidance"),
//         "[ESC] TOO CLOSE — FORCING BACKWARD");

//     return cmd;
// }

//     bool both_sides_free = (left_avg_ > SIDE_FREE_THRESH &&
//                             right_avg_ > SIDE_FREE_THRESH);

//     if (both_sides_free && front_min_ > CAUTION_DIST) {
//         state_ = AvoidState::NAVIGATING;
//         best_front_while_avoiding_ = 999.f;
//         escape_phase_ = 0;
//         clear_count_  = 0;

//         RCLCPP_WARN(rclcpp::get_logger("obstacle_avoidance"),
//             "[ESC] COMPLETE — L=%.2f R=%.2f → NAVIGATING",
//             left_avg_, right_avg_);
//     }
// }
// }
//     // ── AVOIDING phase ──────────────────────────────────────────────────────
// if (state_ == AvoidState::AVOIDING) {
//     float ang_err = gap_angle_;

//     float ang = std::clamp(0.08f * ang_err, -HARD_TURN_RAD_S, HARD_TURN_RAD_S);

//     if (gap_width_deg_ < 40.0f) {
//         ang += (turn_right_ ? -0.5f : 0.5f);
//     }

//     ang += ((rand() % 10) - 5) * 0.02f;

//     if (front_min_ < CAUTION_DIST && std::abs(ang) < 0.4f) {
//         ang = turn_right_ ? -0.4f : 0.4f;
//     }

//     float lin = 0.0f;

//     // TOO CLOSE → FORCE BACKWARD
//     if (front_min_ < 1.0f) {
//         lin = -0.15f;
//         ang = turn_right_ ? -0.6f : 0.6f;
//     }
//     else {
//         float align = std::cos(ang_err * M_PI / 180.0f);

         
//         if (std::abs(ang_err) > 60.0f) {
//             lin = 0.0f;
//         } else {
//             lin = 0.2f * std::max(0.1f, align);
//         }
//     }
//     //  CORNER TRAP DETECTION 
// if (front_min_ < 1.0f && left_avg_ < 1.0f) {
//     lin = -0.2f;
//     ang = -1.0f;
// }

// if (front_min_ < 1.0f && right_avg_ < 1.0f) {
//     lin = -0.2f;
//     ang = 1.0f;
// }

//         if (left_avg_ < 0.7f) {
//         ang = -0.6f;
//     }
//     if (right_avg_ < 0.7f) {
//         ang = 0.6f;
//     }

//     //  GAP COMMIT 
//     if (gap_width_deg_ > 50.0f && front_min_ > 1.2f) {
//         lin = 0.25f;
//         ang *= 0.5f;
//     }

//     //  slow down near obstacles
//     if (front_min_ < 2.0f) {
//         lin *= 0.6f;
//     }

//     cmd.linear.x  = lin;
//     cmd.angular.z = ang;
// }

//   return cmd;
// }

// //  ROVER CONTROLLER NODE

// class RoverController : public rclcpp::Node
// {
// private:
//     RoverState        state_;
//     GlobalPlanner     planner_;
//     ObstacleAvoidance obstacle_;

//     bool gps_received_  = false;
//     bool imu_received_  = false;
//     bool scan_received_ = false;
//     double last_check_x_   = 0.0;
//     double last_check_y_   = 0.0;
//     rclcpp::Time last_check_time_;
//     bool   stuck_check_init_ = false;
//     int    consecutive_escapes_ = 0;
//     bool     post_escape_cooldown_ = false;
//     rclcpp::Time escape_complete_time_;
//     double   escape_exit_heading_  = 0.0;
//     bool prev_escape_state_ = false;

//     rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr  gps_sub_;
//     rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr        imu_sub_;
//     rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr      odom_sub_;
//     rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr  scan_sub_;
//     rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr      depth_sub_;
//     rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr       cmd_pub_;
//     rclcpp::TimerBase::SharedPtr                                   timer_;

// public:
//     RoverController(double goal_lat, double goal_lon)
//     : Node("rover_controller"),
//       planner_(goal_lat, goal_lon)
//     {
//         // GPS
//         gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
//             "/gps/fix", 10,
//             [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
//                 if (msg->status.status < 0) return;
//                 state_.updateFromGPS(*msg);
//                 gps_received_ = true;
//                 double bear = state_.position.bearingTo(planner_.getGoal());
//                 obstacle_.setGoalBearing(static_cast<float>(bear));
//             });

//         // IMU
//         imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
//             "/imu/data", 10,
//             [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
//                 state_.updateFromIMU(*msg);
//                 imu_received_ = true;
//             });

//         // Odometry
//         odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
//             "/odom", 10,
//             [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
//                 state_.updateSpeedFromOdom(*msg);
//                 if (!stuck_check_init_) {
//                     last_check_x_    = msg->pose.pose.position.x;
//                     last_check_y_    = msg->pose.pose.position.y;
//                     last_check_time_ = this->now();
//                     stuck_check_init_ = true;
//                 } else {
//                     double dt = (this->now() - last_check_time_).seconds();
//                     if (dt > 8.0) {
//                         double dx = msg->pose.pose.position.x - last_check_x_;
//                         double dy = msg->pose.pose.position.y - last_check_y_;
//                         double moved = std::sqrt(dx*dx + dy*dy);
//                         if (moved < 0.25 && obstacle_.isObstacle()) {
//                             consecutive_escapes_++;
//                             RCLCPP_ERROR(this->get_logger(),
//                                 "[STUCK] Only moved %.2fm in 8s — forcing ESCAPE #%d",
//                                 moved, consecutive_escapes_);
//                             obstacle_.forceEscape(consecutive_escapes_);
//                         } else {
//                             consecutive_escapes_ = 0;
//                         }
//                         last_check_x_    = msg->pose.pose.position.x;
//                         last_check_y_    = msg->pose.pose.position.y;
//                         last_check_time_ = this->now();
//                     }
//                 }
//             });

//         // LiDAR
//         scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
//             "/scan", 10,
//             [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
//                 obstacle_.processScan(msg);
//                 scan_received_ = true;
//             });

//         // Stereo depth
//         depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
//             "/stereo/depth", 10,
//             [this](const sensor_msgs::msg::Image::SharedPtr msg) {
//                 obstacle_.processDepth(msg);
//             });

//         // Command publisher
//         cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

//         // 20 Hz control loop
//         timer_ = create_wall_timer(
//             std::chrono::milliseconds(50),
//             std::bind(&RoverController::controlLoop, this));

//         RCLCPP_INFO(this->get_logger(),
//             "RoverController started — goal (%.6f, %.6f)", goal_lat, goal_lon);
//     }

//     void controlLoop()
//     {
//          constexpr float SLOW_DIST    = 3.5f;
//     constexpr float CAUTION_DIST = 1.5f;
//     constexpr float BRAKE_DIST   = 0.5f;
//         RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
//             "[CTRL] obs=%d front=%.2f state=%d | scan=%d gps=%d imu=%d",
//             (int)obstacle_.isObstacle(), obstacle_.getFront(),
//             (int)obstacle_.getState(),
//             (int)scan_received_, (int)gps_received_, (int)imu_received_);

       
// bool currently_escaping = (obstacle_.getState() == AvoidState::ESCAPING);
// if (prev_escape_state_ && !currently_escaping) {
//     post_escape_cooldown_  = true;
//     escape_complete_time_  = this->now();
//     escape_exit_heading_   = state_.heading_deg;
//     obstacle_.suppressGoalPullFor(40);
//     RCLCPP_WARN(this->get_logger(),
//         "[COOL] Escape complete, locking heading %.1f° for 4s",
//         escape_exit_heading_);
// }
// prev_escape_state_ = currently_escaping;

//         if (!gps_received_ || !imu_received_ || !scan_received_)
//             return;

//         double dist = state_.position.distanceTo(planner_.getGoal());
//         if (dist < 0.5) {
//             RCLCPP_INFO(this->get_logger(), "[CTRL] Goal reached — STOP");
//             geometry_msgs::msg::Twist stop;
//             stop.linear.x  = 0.0;
//             stop.angular.z = 0.0;
//             cmd_pub_->publish(stop);
//             return;
//         }

//         if (planner_.isGoalReached()) {
//             geometry_msgs::msg::Twist stop;
//             cmd_pub_->publish(stop);
//             return;
//         }

//         if (!rclcpp::ok()) return;

//         geometry_msgs::msg::Twist cmd;

//         if (obstacle_.isObstacle()) {
//             cmd = obstacle_.computeAvoidance();
//         } else {
//             // Post-escape cooldown: drive straight, ignore goal
//             if (post_escape_cooldown_) {
//                 double cooldown_elapsed = (this->now() - escape_complete_time_).seconds();
//                 if (cooldown_elapsed < 4.0) {
//                     if (obstacle_.isObstacle()) {
//                         post_escape_cooldown_ = false;
//                         cmd = obstacle_.computeAvoidance();
//                     } else {
//                         cmd.linear.x  = 0.15;
//                         cmd.angular.z = 0.0;
//                     }
//                     RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
//                         "[COOL] Post-escape drive %.1f/4.0s", cooldown_elapsed);
//                 } else {
//                     post_escape_cooldown_ = false;
//                     RCLCPP_WARN(this->get_logger(), "[COOL] Cooldown done — resuming navigation");
//                     cmd = planner_.computeCommand(state_);
//                 }
//             } else {
//                 cmd = planner_.computeCommand(state_);
//             }

//             double bear_fresh = state_.position.bearingTo(planner_.getGoal());
//             double rel_bear   = GlobalPlanner::normaliseAngle(bear_fresh - state_.heading_deg + 180.0);
//             obstacle_.setGoalBearing(static_cast<float>(rel_bear));

//             RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
//                 "[NAV] dist=%.2fm bear=%.1f° head=%.1f° err=%.1f° lin=%.2f ang=%.2f",
//                 dist, bear_fresh, state_.heading_deg, rel_bear,
//                 cmd.linear.x, cmd.angular.z);
//         }



// float front = obstacle_.getFront();
// if (!obstacle_.isObstacle() && front < SLOW_DIST) {
//     double scale = (front - CAUTION_DIST) / (SLOW_DIST - CAUTION_DIST);
//     scale = std::max(0.0, std::min(1.0, scale));
//     cmd.linear.x *= scale;
// }

// // Hard brake 
// if (front < BRAKE_DIST) {
//     cmd.linear.x  = -0.05;
//     cmd.angular.z = obstacle_.getTurnRight() ? -1.0 : 1.0;
// }

// cmd_pub_->publish(cmd);
//     }
// };

// //  MAIN

// int main(int argc, char** argv)
// {
//     rclcpp::init(argc, argv);

//     double goal_lat = 0.0, goal_lon = 0.0;
//     if (argc == 3) {
//         goal_lat = std::stod(argv[1]);
//         goal_lon = std::stod(argv[2]);
//     } else {
//         RCLCPP_WARN(rclcpp::get_logger("main"),
//             "Usage: rover_controller <lat> <lon> — using (0,0) as goal");
//     }

//     auto node = std::make_shared<RoverController>(goal_lat, goal_lon);
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }




#include <new>
#include "rclcpp/rclcpp.hpp"
#include "robot/global_planner.hpp"
#include "robot/rover_state.hpp"
#include "robot/obstacle_avoidance.hpp"

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>
#include <limits>
#include <numeric>
#include <vector>
#include <chrono>


//  GLOBAL PLANNER

GlobalPlanner::GlobalPlanner(double goal_lat, double goal_lon)
: goal_(goal_lat, goal_lon)
{
}

geometry_msgs::msg::Twist
GlobalPlanner::computeCommand(const RoverState& state)
{
    geometry_msgs::msg::Twist cmd;

    double distance_m   = state.position.distanceTo(goal_);
    double goal_bearing = state.position.bearingTo(goal_);

    if (distance_m < GOAL_TOLERANCE_M || goal_reached_) {
        if (!goal_reached_) {
            RCLCPP_INFO(rclcpp::get_logger("global_planner"),
                "[GOAL] Reached! dist=%.2fm — stopping", distance_m);
            goal_reached_ = true;
        }
        cmd.linear.x  = 0.0;
        cmd.angular.z = 0.0;
        return cmd;
    }

    double heading_error = normaliseAngle(goal_bearing - state.heading_deg + 180.0);
    double angular_z     = clamp(-KP_ANGULAR * heading_error, MAX_ANGULAR_RADS);

    if (std::abs(heading_error) > 90.0) {
        angular_z = (heading_error > 0) ? -MAX_ANGULAR_RADS : MAX_ANGULAR_RADS;
    }

    double dist_scale = std::min(1.0, distance_m / 3.0);
    double alignment  = std::cos(heading_error * M_PI / 180.0);

    double linear_x;
    if (std::abs(heading_error) > 60.0) {
    linear_x = 0.0;
    cmd.angular.z = clamp(-KP_ANGULAR * heading_error, MAX_ANGULAR_RADS);
    // Force max turn when very misaligned
    if (std::abs(heading_error) > 90.0)
        cmd.angular.z = (heading_error > 0) ? -MAX_ANGULAR_RADS : MAX_ANGULAR_RADS;
    cmd.linear.x = linear_x;
    return cmd;
} else {
    linear_x = LINEAR_SPEED_MPS * std::max(0.2, alignment) * dist_scale;
}

    if (linear_x > 0.0)
        linear_x = std::max(0.05, linear_x);

    cmd.linear.x  = linear_x;
    cmd.angular.z = angular_z;
    return cmd;
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


//  OBSTACLE AVOIDANCE

void ObstacleAvoidance::processScan(
    const sensor_msgs::msg::LaserScan::SharedPtr& msg)
{
    const int n  = static_cast<int>(msg->ranges.size());
    angle_min_   = msg->angle_min;
    angle_inc_   = msg->angle_increment;
    ranges_      = msg->ranges;

    auto deg2idx = [&](float deg) -> int {
        float rad = deg * static_cast<float>(M_PI) / 180.0f;
        int   idx = static_cast<int>(std::round((rad - angle_min_) / angle_inc_));
        return std::max(0, std::min(n - 1, idx));
    };

    // Front cone minimum
    int lo = deg2idx(-FRONT_FOV_DEG);
    int hi = deg2idx( FRONT_FOV_DEG);
    if (lo > hi) std::swap(lo, hi);

    front_min_       = 999.f;
    front_min_angle_ = 0.0f;
    for (int i = lo; i <= hi; ++i) {
        float r = ranges_[i];
        if (std::isfinite(r) && r > 0.05f && r < front_min_) {
            front_min_       = r;
            front_min_angle_ = idx2deg(i);
        }
    }

    // Left/right side averages
    auto sideAvg = [&](float from_deg, float to_deg) -> float {
        int a = deg2idx(from_deg);
        int b = deg2idx(to_deg);
        if (a > b) std::swap(a, b);
        float sum = 0.f; int cnt = 0;
        for (int i = a; i <= b; ++i) {
            float r = ranges_[i];
            if (std::isfinite(r) && r > 0.05f) { sum += r; cnt++; }
        }
        return (cnt > 0) ? sum / cnt : 999.f;
    };

    left_avg_       = sideAvg( 10.f,  90.f);
    right_avg_      = sideAvg(-90.f, -10.f);
    back_left_avg_  = sideAvg( 90.f, 170.f);
    back_right_avg_ = sideAvg(-170.f, -90.f);
    rear_min_       = sideAvg(150.f, 180.f);
    rear_min_       = std::min(rear_min_, sideAvg(-180.f, -150.f));

    // Find best gap direction
    {
        const float SECTOR_DEG = 15.0f;
        float best_gap   = goal_bearing_;
        float best_score = 1e9f;
        const float OPEN_THRESH = 2.0f;
        const float SEARCH_FOV  = 180.0f;

        if (suppress_goal_pull_) {
            if (--suppress_scan_count_ <= 0) suppress_goal_pull_ = false;
        }

        for (float centre = -SEARCH_FOV; centre <= SEARCH_FOV; centre += SECTOR_DEG) {
            int a2 = deg2idx(centre - SECTOR_DEG * 0.5f);
            int b2 = deg2idx(centre + SECTOR_DEG * 0.5f);
            if (a2 > b2) std::swap(a2, b2);
            float sum2 = 0.f; int cnt2 = 0;
            for (int i = a2; i <= b2; ++i) {
                float r = ranges_[i];
                if (std::isfinite(r) && r > 0.05f) { sum2 += r; cnt2++; }
            }
            float avg_clear = (cnt2 > 0) ? sum2 / cnt2 : 999.f;
            if (avg_clear < OPEN_THRESH) continue;

            float diff = std::abs(centre - goal_bearing_);
            if (diff > 180.f) diff = 360.f - diff;
            float goal_penalty = suppress_goal_pull_ ? (diff / 120.0f) : (diff / 45.0f);
            float score = goal_penalty - (avg_clear * 0.8f);
            if (score < best_score) {
                best_score = score;
                best_gap   = centre;
            }
        }
        gap_angle_ = best_gap;

        float width = 0.0f;
        for (float c = best_gap - SEARCH_FOV; c <= best_gap + SEARCH_FOV; c += SECTOR_DEG) {
            int a2 = deg2idx(c - SECTOR_DEG * 0.5f);
            int b2 = deg2idx(c + SECTOR_DEG * 0.5f);
            if (a2 > b2) std::swap(a2, b2);
            float sum2 = 0.f; int cnt2 = 0;
            for (int i = a2; i <= b2; ++i) {
                float r = ranges_[i];
                if (std::isfinite(r) && r > 0.05f) { sum2 += r; cnt2++; }
            }
            float avg_clear = (cnt2 > 0) ? sum2 / cnt2 : 999.f;
            if (avg_clear > ESCAPE_CLEAR_THRESH)
                width += SECTOR_DEG;
        }
        gap_width_deg_ = width;
    }

    if (!logged_once_) {
        RCLCPP_INFO(rclcpp::get_logger("obstacle_avoidance"),
            "[SCAN] n=%d angle_min=%.2f inc=%.4f front_fov=±%.0f° front=%.2f L=%.2f R=%.2f",
            n, angle_min_, angle_inc_, FRONT_FOV_DEG,
            front_min_, left_avg_, right_avg_);
        logged_once_ = true;
    }

    updateState();
}

void ObstacleAvoidance::processDepth(
    const sensor_msgs::msg::Image::SharedPtr& msg)
{
    if (msg->encoding != "32FC1") return;

    const float* data = reinterpret_cast<const float*>(msg->data.data());
    const int W = static_cast<int>(msg->width);
    const int H = static_cast<int>(msg->height);

    int col0 = W * 4 / 10, col1 = W * 6 / 10;
    int row0 = H * 2 / 10, row1 = H * 8 / 10;

    float min_depth = 999.f;
    for (int r = row0; r < row1; ++r)
        for (int c = col0; c < col1; ++c) {
            float d = data[r * W + c];
            if (std::isfinite(d) && d > 0.05f)
                min_depth = std::min(min_depth, d);
        }

    if (min_depth < front_min_) {
        RCLCPP_INFO(rclcpp::get_logger("obstacle_avoidance"),
            "[DEPTH] stereo=%.2f overrides lidar=%.2f", min_depth, front_min_);
        front_min_ = min_depth;
        updateState();
    }
}

void ObstacleAvoidance::updateState()
{
    using SC = std::chrono::steady_clock;
    const auto now = SC::now();

    bool obstacle_ahead;
    if (state_ == AvoidState::NAVIGATING)
        obstacle_ahead = (front_min_ < CAUTION_DIST);
    else
        obstacle_ahead = (front_min_ < CLEAR_DIST);

    if (!obstacle_ahead && state_ == AvoidState::AVOIDING) {
        clear_count_++;
        if (clear_count_ >= 2) {
            RCLCPP_INFO(rclcpp::get_logger("obstacle_avoidance"),
                "[OBS] Path clear — NAVIGATING (front=%.2f)", front_min_);
            state_ = AvoidState::NAVIGATING;
            best_front_while_avoiding_ = 999.f;
            clear_count_ = 0;
            full_escape_attempts_ = 0;
        }
        return;
    } else {
        clear_count_ = 0;
    }

    if (obstacle_ahead && state_ == AvoidState::NAVIGATING) {
        state_                     = AvoidState::AVOIDING;
        avoiding_since_            = now;
        best_front_while_avoiding_ = front_min_;
        turn_right_ = ((right_avg_ + back_right_avg_) > (left_avg_ + back_left_avg_));

        float dir = turn_right_ ? -1.0f : 1.0f;
        if (dir * last_avoid_direction_ > 0) {
            same_dir_count_++;
            if (same_dir_count_ > 2) {
                turn_right_ = !turn_right_;
                same_dir_count_ = 0;
                RCLCPP_WARN(rclcpp::get_logger("obstacle_avoidance"),
                    "[OBS] Same direction 3x — flipping turn");
            }
        } else {
            same_dir_count_ = 0;
        }
        last_avoid_direction_ = dir;

        suppress_goal_pull_  = true;
        suppress_scan_count_ = 8;

        RCLCPP_WARN(rclcpp::get_logger("obstacle_avoidance"),
            "[OBS] OBSTACLE front=%.2f L=%.2f R=%.2f — turning %s",
            front_min_, left_avg_, right_avg_,
            turn_right_ ? "RIGHT" : "LEFT");
        return;
    }

    if (state_ == AvoidState::AVOIDING) {
        if (front_min_ > best_front_while_avoiding_ + STUCK_IMPROVE_M) {
            best_front_while_avoiding_ = front_min_;
            avoiding_since_            = now;
        }

        double elapsed = std::chrono::duration<double>(now - avoiding_since_).count();

        if (elapsed > STUCK_TIMEOUT_S) {
            state_        = AvoidState::ESCAPING;
            escape_start_ = now;
            escape_phase_ = 0;
            clear_count_  = 0;

            full_escape_attempts_++;

            // KEY FIX: flip direction on every escape attempt
            turn_right_ = !turn_right_;
            suppress_goal_pull_ = false;

            RCLCPP_ERROR(rclcpp::get_logger("obstacle_avoidance"),
                "[STRATEGY] Escape #%d — now turning %s",
                full_escape_attempts_, turn_right_ ? "RIGHT" : "LEFT");

            if (full_escape_attempts_ >= 2) {
                gap_angle_ = goal_bearing_;
                RCLCPP_ERROR(rclcpp::get_logger("obstacle_avoidance"),
                    "[STRATEGY] Forcing GOAL direction after %d attempts",
                    full_escape_attempts_);
            }

            RCLCPP_ERROR(rclcpp::get_logger("obstacle_avoidance"),
                "[OBS] LOCAL MINIMA — stuck %.1fs, best_front=%.2f → ESCAPING",
                elapsed, best_front_while_avoiding_);
        }
    }
}

geometry_msgs::msg::Twist ObstacleAvoidance::computeAvoidance()
{
    auto clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
    using SC   = std::chrono::steady_clock;
    geometry_msgs::msg::Twist cmd;

    // ── ESCAPING ────────────────────────────────────────────────────────────
    if (state_ == AvoidState::ESCAPING) {
        double elapsed = std::chrono::duration<double>(
            SC::now() - escape_start_).count();

        // Phase 0: Reverse
        if (escape_phase_ == 0) {
            float side_diff = right_avg_ - left_avg_;
            float steer = 0.0f;
            if (std::abs(side_diff) > 1.0f)
                steer = (side_diff < 0) ? +0.3f : -0.3f;

            if (rear_min_ < 1.0f) {
                cmd.linear.x  = 0.0;
                cmd.angular.z = (turn_right_ ? -0.8f : 0.8f);
                RCLCPP_WARN(rclcpp::get_logger("obstacle_avoidance"),
                    "[ESC] REAR TOO CLOSE — rotating (rear=%.2f)", rear_min_);
                return cmd;
            }

            if (left_avg_ < 0.8f || right_avg_ < 0.8f) {
                cmd.linear.x  = 0.0;
                cmd.angular.z = (turn_right_ ? -0.8f : 0.8f);
                RCLCPP_WARN(rclcpp::get_logger("obstacle_avoidance"),
                    "[ESC] SIDE TOO CLOSE — rotating");
                return cmd;
            }

            cmd.linear.x  = ESCAPE_REVERSE_MPS;
            cmd.angular.z = steer;

            RCLCPP_WARN_THROTTLE(rclcpp::get_logger("obstacle_avoidance"),
                *clock_, 500, "[ESC] BACKING %.1f/%.1fs steer=%.1f L=%.2f R=%.2f",
                elapsed, ESCAPE_REVERSE_S, steer, left_avg_, right_avg_);

            if (elapsed > ESCAPE_REVERSE_S) {
                escape_phase_ = 1;
                escape_start_ = SC::now();
            }

        // Phase 1: Spin until gap opens
        } else if (escape_phase_ == 1) {
            cmd.linear.x  = 0.0;

            // KEY FIX: re-evaluate best turn direction at start of spin
            if (std::chrono::duration<double>(SC::now() - escape_start_).count() < 0.3) {
                turn_right_ = (right_avg_ > left_avg_);
            }

            const float spin_sign = turn_right_ ? -1.f : 1.f;
            cmd.angular.z = spin_sign * HARD_TURN_RAD_S;

            RCLCPP_WARN_THROTTLE(rclcpp::get_logger("obstacle_avoidance"),
                *clock_, 500, "[ESC] SPINNING %s — front=%.2f L=%.2f R=%.2f need>%.2f",
                turn_right_ ? "RIGHT" : "LEFT",
                front_min_, left_avg_, right_avg_, ESCAPE_CLEAR_THRESH);

            double max_spin   = (3.0 * M_PI) / HARD_TURN_RAD_S;
            double min_spin_s = M_PI / HARD_TURN_RAD_S;

            bool rear_clear  = (rear_min_ > ESCAPE_CLEAR_THRESH && elapsed > min_spin_s);
            bool gap_found   = rear_clear ||
                               (front_min_ > ESCAPE_CLEAR_THRESH &&
                                gap_width_deg_ >= ESCAPE_GAP_WIDTH_DEG &&
                                elapsed > min_spin_s);
            bool spin_timeout = (elapsed > max_spin);

            if (gap_found || spin_timeout) {
                if (front_min_ < DANGER_DIST) {
                    cmd.linear.x  = -0.15f;
                    cmd.angular.z = turn_right_ ? -0.5f : 0.5f;
                    return cmd;
                } else {
                    escape_phase_ = 2;
                    escape_start_ = SC::now();
                    RCLCPP_WARN(rclcpp::get_logger("obstacle_avoidance"),
                        "[ESC] Gap found (front=%.2f gap_w=%.0f°) — DRIVING",
                        front_min_, gap_width_deg_);
                }
            }

        // Phase 2: Drive through gap
        } else if (escape_phase_ == 2) {
            if (front_min_ < 0.7f) {
                cmd.linear.x  = 0.0;
                cmd.angular.z = (turn_right_ ? -0.8f : 0.8f);
                RCLCPP_WARN(rclcpp::get_logger("obstacle_avoidance"),
                    "[ESC] TOO CLOSE — rotating (front=%.2f)", front_min_);
                return cmd;
            }

            float side_diff = left_avg_ - right_avg_;
            float ang = std::clamp(0.2f * side_diff, -HARD_TURN_RAD_S, HARD_TURN_RAD_S);
            ang += (turn_right_ ? -0.3f : 0.3f);
            ang  = std::clamp(ang, -HARD_TURN_RAD_S, HARD_TURN_RAD_S);

            cmd.linear.x  = 0.15f;
            cmd.angular.z = ang;

            RCLCPP_WARN_THROTTLE(rclcpp::get_logger("obstacle_avoidance"),
                *clock_, 500, "[ESC] DRIVING gap=%.1f° L=%.2f R=%.2f front=%.2f",
                gap_angle_, left_avg_, right_avg_, front_min_);

            if (front_min_ < DANGER_DIST) {
                // Blocked again — go back to phase 0
                escape_phase_ = 0;
                escape_start_ = SC::now();
                turn_right_   = !turn_right_;   // KEY FIX: flip before retry
                RCLCPP_WARN(rclcpp::get_logger("obstacle_avoidance"),
                    "[ESC] Blocked in phase2 — restarting escape, flipping to %s",
                    turn_right_ ? "RIGHT" : "LEFT");
                cmd.linear.x  = ESCAPE_REVERSE_MPS;
                cmd.angular.z = 0.0;
                return cmd;
            }

            bool both_sides_free = (left_avg_ > SIDE_FREE_THRESH &&
                                    right_avg_ > SIDE_FREE_THRESH);

            if (both_sides_free && front_min_ > CAUTION_DIST) {
                state_        = AvoidState::NAVIGATING;
                best_front_while_avoiding_ = 999.f;
                escape_phase_ = 0;
                clear_count_  = 0;
                full_escape_attempts_ = 0;
                RCLCPP_WARN(rclcpp::get_logger("obstacle_avoidance"),
                    "[ESC] COMPLETE — L=%.2f R=%.2f → NAVIGATING",
                    left_avg_, right_avg_);
            }
        }
    }

    // ── AVOIDING ────────────────────────────────────────────────────────────
    if (state_ == AvoidState::AVOIDING) {
        float ang_err = gap_angle_;
        float ang = std::clamp(0.08f * ang_err, -HARD_TURN_RAD_S, HARD_TURN_RAD_S);

        if (gap_width_deg_ < 40.0f)
            ang += (turn_right_ ? -0.5f : 0.5f);

        // small random jitter to break symmetry
        ang += ((rand() % 10) - 5) * 0.02f;

        if (front_min_ < CAUTION_DIST && std::abs(ang) < 0.4f)
            ang = turn_right_ ? -0.4f : 0.4f;

        float lin = 0.0f;

        if (front_min_ < 1.0f) {
            lin = -0.15f;
            ang = turn_right_ ? -0.6f : 0.6f;
        } else {
            float align = std::cos(ang_err * M_PI / 180.0f);
            lin = (std::abs(ang_err) > 60.0f) ? 0.0f : 0.2f * std::max(0.1f, align);
        }

        // corner trap detection
        if (front_min_ < 1.0f && left_avg_ < 1.0f)  { lin = -0.2f; ang = -1.0f; }
        if (front_min_ < 1.0f && right_avg_ < 1.0f) { lin = -0.2f; ang =  1.0f; }

        if (left_avg_  < 0.7f) ang = -0.6f;
        if (right_avg_ < 0.7f) ang =  0.6f;

        if (gap_width_deg_ > 50.0f && front_min_ > 1.2f) { lin = 0.25f; ang *= 0.5f; }
        if (front_min_ < 2.0f) lin *= 0.6f;

        cmd.linear.x  = lin;
        cmd.angular.z = ang;
    }

    return cmd;
}


//  ROVER CONTROLLER NODE

class RoverController : public rclcpp::Node
{
private:
    RoverState        state_;
    GlobalPlanner     planner_;
    ObstacleAvoidance obstacle_;

    bool gps_received_  = false;
    bool imu_received_  = false;
    bool scan_received_ = false;

    double last_check_x_     = 0.0;
    double last_check_y_     = 0.0;
    rclcpp::Time last_check_time_;
    bool   stuck_check_init_ = false;
    int    consecutive_escapes_ = 0;

    bool     post_escape_cooldown_ = false;
    rclcpp::Time escape_complete_time_;
    bool prev_escape_state_ = false;

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr  gps_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr        imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr      odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr  scan_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr      depth_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr       cmd_pub_;
    rclcpp::TimerBase::SharedPtr                                   timer_;

public:
    RoverController(double goal_lat, double goal_lon)
    : Node("rover_controller"),
      planner_(goal_lat, goal_lon)
    {
        gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
            "/gps/fix", 10,
            [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
                if (msg->status.status < 0) return;
                state_.updateFromGPS(*msg);
                gps_received_ = true;
                double bear = state_.position.bearingTo(planner_.getGoal());
                obstacle_.setGoalBearing(static_cast<float>(bear));
            });

        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10,
            [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
                state_.updateFromIMU(*msg);
                imu_received_ = true;
            });

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                state_.updateSpeedFromOdom(*msg);

                if (!stuck_check_init_) {
                    last_check_x_    = msg->pose.pose.position.x;
                    last_check_y_    = msg->pose.pose.position.y;
                    last_check_time_ = this->now();
                    stuck_check_init_ = true;
                } else {
                    double dt = (this->now() - last_check_time_).seconds();
                    if (dt > 8.0) {
                        double dx    = msg->pose.pose.position.x - last_check_x_;
                        double dy    = msg->pose.pose.position.y - last_check_y_;
                        double moved = std::sqrt(dx*dx + dy*dy);

                        if (moved < 0.25 && obstacle_.isObstacle()) {
                            consecutive_escapes_++;
                            RCLCPP_ERROR(this->get_logger(),
                                "[STUCK] Only moved %.2fm in 8s — forcing ESCAPE #%d",
                                moved, consecutive_escapes_);
                            obstacle_.forceEscape(consecutive_escapes_);
                        } else {
                            consecutive_escapes_ = 0;
                        }

                        last_check_x_    = msg->pose.pose.position.x;
                        last_check_y_    = msg->pose.pose.position.y;
                        last_check_time_ = this->now();
                    }
                }
            });

        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                obstacle_.processScan(msg);
                scan_received_ = true;
            });

        depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/stereo/depth", 10,
            [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                obstacle_.processDepth(msg);
            });

        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&RoverController::controlLoop, this));

        RCLCPP_INFO(this->get_logger(),
            "RoverController started — goal (%.6f, %.6f)", goal_lat, goal_lon);
    }

    void controlLoop()
    {
        constexpr float SLOW_DIST    = 3.5f;
        constexpr float CAUTION_DIST = 1.5f;
        constexpr float BRAKE_DIST   = 0.5f;

        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "[CTRL] obs=%d front=%.2f state=%d | scan=%d gps=%d imu=%d",
            (int)obstacle_.isObstacle(), obstacle_.getFront(),
            (int)obstacle_.getState(),
            (int)scan_received_, (int)gps_received_, (int)imu_received_);

        bool currently_escaping = (obstacle_.getState() == AvoidState::ESCAPING);
        if (prev_escape_state_ && !currently_escaping) {
            post_escape_cooldown_ = true;
            escape_complete_time_ = this->now();
            obstacle_.suppressGoalPullFor(40);
            RCLCPP_WARN(this->get_logger(), "[COOL] Escape complete — coasting 4s");
        }
        prev_escape_state_ = currently_escaping;

        if (!gps_received_ || !imu_received_ || !scan_received_)
            return;

        double dist = state_.position.distanceTo(planner_.getGoal());
        if (dist < 0.5 || planner_.isGoalReached()) {
            RCLCPP_INFO(this->get_logger(), "[CTRL] Goal reached — STOP");
            geometry_msgs::msg::Twist stop;
            cmd_pub_->publish(stop);
            return;
        }

        if (!rclcpp::ok()) return;

        geometry_msgs::msg::Twist cmd;

        if (obstacle_.isObstacle()) {
            cmd = obstacle_.computeAvoidance();
        } else {
            if (post_escape_cooldown_) {
                double elapsed = (this->now() - escape_complete_time_).seconds();
                if (elapsed < 4.0) {
                    if (obstacle_.isObstacle()) {
                        post_escape_cooldown_ = false;
                        cmd = obstacle_.computeAvoidance();
                    } else {
                        cmd = planner_.computeCommand(state_);
                        cmd.linear.x = std::min(cmd.linear.x, 0.1); 
                    }
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "[COOL] Post-escape drive %.1f/4.0s", elapsed);
                } else {
                    post_escape_cooldown_ = false;
                    cmd = planner_.computeCommand(state_);
                }
            } else {
                cmd = planner_.computeCommand(state_);
            }

            double bear_fresh = state_.position.bearingTo(planner_.getGoal());
            double rel_bear   = GlobalPlanner::normaliseAngle(bear_fresh - state_.heading_deg + 180.0);
            obstacle_.setGoalBearing(static_cast<float>(rel_bear));

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "[NAV] dist=%.2fm bear=%.1f° head=%.1f° err=%.1f° lin=%.2f ang=%.2f",
                dist, bear_fresh, state_.heading_deg, rel_bear,
                cmd.linear.x, cmd.angular.z);
        }

        float front = obstacle_.getFront();
        if (!obstacle_.isObstacle() && front < SLOW_DIST) {
            double scale = (front - CAUTION_DIST) / (SLOW_DIST - CAUTION_DIST);
            scale = std::max(0.0, std::min(1.0, scale));
            cmd.linear.x *= scale;
        }

        if (front < BRAKE_DIST) {
            cmd.linear.x  = -0.05;
            cmd.angular.z = obstacle_.getTurnRight() ? -1.0 : 1.0;
        }

        cmd_pub_->publish(cmd);
    }
};


//  MAIN

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    double goal_lat = 0.0, goal_lon = 0.0;
    if (argc == 3) {
        goal_lat = std::stod(argv[1]);
        goal_lon = std::stod(argv[2]);
    } else {
        RCLCPP_WARN(rclcpp::get_logger("main"),
            "Usage: rover_controller <lat> <lon> — using (0,0) as goal");
    }

    auto node = std::make_shared<RoverController>(goal_lat, goal_lon);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}