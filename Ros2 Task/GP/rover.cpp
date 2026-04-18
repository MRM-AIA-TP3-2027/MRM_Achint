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

    if (std::abs(heading_error) > 120.0) {
        cmd.linear.x  = 0.0;
        cmd.angular.z = angular_z;
        return cmd;
    }

    double dist_scale = std::min(1.0, distance_m / 3.0);
    double alignment  = std::cos(heading_error * M_PI / 180.0);
    double linear_x   = LINEAR_SPEED_MPS * std::max(0.0, alignment) * dist_scale;

    linear_x = std::max(0.03, linear_x);
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

    //  Front cone minimum 
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

    //  Left/right side averages 
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

    left_avg_  = sideAvg( 10.f,  90.f);
    right_avg_ = sideAvg(-90.f, -10.f);

    // ── Find best gap direction 
    {
        const float SECTOR_DEG  = 15.0f;   // width of each candidate sector
        const float OPEN_THRESH = 1.5f;    // metres — sector must be this clear
        const float SEARCH_FOV  = 90.0f;   // ±90° around front

        float best_gap   = goal_bearing_;   
        float best_score = 1e9f;

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
            if (diff < best_score) {
                best_score = diff;
                best_gap   = centre;
            }
        }
        gap_angle_ = best_gap;
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

//  updateState 
void ObstacleAvoidance::updateState()
{
    using SC = std::chrono::steady_clock;
    const auto now = SC::now();

    bool obstacle_ahead;
    if (state_ == AvoidState::NAVIGATING)
        obstacle_ahead = (front_min_ < CAUTION_DIST);
    else
        obstacle_ahead = (front_min_ < CLEAR_DIST);   // hysteresis — stay avoiding longer

    //  NAVIGATING → clear 
    if (!obstacle_ahead && state_ == AvoidState::AVOIDING) {
    clear_count_++;
    if (clear_count_ >= 2) {
        RCLCPP_INFO(rclcpp::get_logger("obstacle_avoidance"),
            "[OBS] Path clear — NAVIGATING (front=%.2f)", front_min_);
        state_ = AvoidState::NAVIGATING;
        best_front_while_avoiding_ = 999.f;
        clear_count_ = 0;
    }
    return;
} else {
    clear_count_ = 0;
}

    //  NAVIGATING → obstacle 
    if (obstacle_ahead && state_ == AvoidState::NAVIGATING) {
        state_                     = AvoidState::AVOIDING;
        avoiding_since_            = now;
        best_front_while_avoiding_ = front_min_;
        turn_right_                = (right_avg_ > left_avg_);
        RCLCPP_WARN(rclcpp::get_logger("obstacle_avoidance"),
            "[OBS] OBSTACLE front=%.2f L=%.2f R=%.2f — turning %s",
            front_min_, left_avg_, right_avg_,
            turn_right_ ? "RIGHT" : "LEFT");
        return;
    }

    //  AVOIDING — check for stuck / local minima 
    
    if (state_ == AvoidState::AVOIDING) {
        if (front_min_ > best_front_while_avoiding_ + STUCK_IMPROVE_M) {
            best_front_while_avoiding_ = front_min_;
            avoiding_since_            = now;   // reset stuck timer on progress
        }

        double elapsed = std::chrono::duration<double>(now - avoiding_since_).count();
        if (elapsed > STUCK_TIMEOUT_S) {
            state_        = AvoidState::ESCAPING;
            escape_start_ = now;
            escape_phase_ = 0;
            clear_count_  = 0;
            turn_right_   = !turn_right_;
            RCLCPP_ERROR(rclcpp::get_logger("obstacle_avoidance"),
                "[OBS] LOCAL MINIMA — stuck %.1fs, best_front=%.2f → ESCAPING, flipping to %s",
                elapsed, best_front_while_avoiding_,
                turn_right_ ? "RIGHT" : "LEFT");
        }
    }

    //  ESCAPING 
   
}

//  computeAvoidance 
geometry_msgs::msg::Twist ObstacleAvoidance::computeAvoidance()
{
    auto clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
    using SC   = std::chrono::steady_clock;
    geometry_msgs::msg::Twist cmd;

    //  ESCAPE phase — reverse then hard-turn to break local minima
    if (state_ == AvoidState::ESCAPING) {
        double elapsed = std::chrono::duration<double>(
            SC::now() - escape_start_).count();

        const float turn_sign = turn_right_ ? -1.f : 1.f;

        if (escape_phase_ == 0) {
            cmd.linear.x  = ESCAPE_REVERSE_MPS;   
            cmd.angular.z = 0.0;
            RCLCPP_WARN_THROTTLE(rclcpp::get_logger("obstacle_avoidance"),
                *clock, 500,
                "[ESC] REVERSING %.1f / %.1fs", elapsed, ESCAPE_REVERSE_S);
            if (elapsed > ESCAPE_REVERSE_S) {
                escape_phase_ = 1;
                escape_start_ = SC::now();
            }
        } else {
            cmd.linear.x  = 0.0;
            cmd.angular.z = turn_sign * HARD_TURN_RAD_S;
            RCLCPP_WARN_THROTTLE(rclcpp::get_logger("obstacle_avoidance"),
                *clock, 500,
                "[ESC] TURNING %.1f / %.1fs toward %s",
                elapsed, ESCAPE_TURN_S, turn_right_ ? "RIGHT" : "LEFT");
            if (elapsed > ESCAPE_TURN_S) {
                state_                     = AvoidState::AVOIDING;
                avoiding_since_            = SC::now();
                best_front_while_avoiding_ = front_min_;
                    escape_phase_ = 0;
                RCLCPP_WARN(rclcpp::get_logger("obstacle_avoidance"),
                    "[ESC] Done — back to AVOIDING (front=%.2f)", front_min_);
            }
        }
        return cmd;
    }
    if (front_min_ < DANGER_DIST && state_ == AvoidState::AVOIDING) {
        state_        = AvoidState::ESCAPING;
        escape_start_ = SC::now();
        escape_phase_ = 0;
        clear_count_  = 0;
        turn_right_   = (right_avg_ > left_avg_);
        RCLCPP_ERROR(rclcpp::get_logger("obstacle_avoidance"),
            "[OBS] DANGER DIST hit (%.2fm) — ESCAPING immediately toward %s",
            front_min_, turn_right_ ? "RIGHT" : "LEFT");
        // Return stop for this cycle, escape runs next cycle
        cmd.linear.x  = 0.0;
        cmd.angular.z = 0.0;
        return cmd;
    }

    //  AVOIDING phase — gap-based steering
    //  gap_angle_ is computed fresh each scan in processScan()
    const float turn_sign = turn_right_ ? -1.f : 1.f;

    // Fall back to side-preference turn if no useful gap found.
    float ang_err = gap_angle_;   // degrees

    // Proportional gain – stronger so the robot actually turns
    float ang = std::clamp(0.04f * ang_err, -HARD_TURN_RAD_S, HARD_TURN_RAD_S);

    // If front is very blocked and gap finder defaulted to goal bearing,
    // override with a hard turn toward the open side.
    if (front_min_ < DANGER_DIST) {
        ang = turn_sign * HARD_TURN_RAD_S;
    }

    // Linear speed 
    float lin;
    if (front_min_ < DANGER_DIST) {
        lin = 0.03f;
    } else if (std::abs(ang_err) < 25.0f) {
        float align_factor = 1.0f - (std::abs(ang_err) / 25.0f);
        lin = LINEAR_SPEED_MPS * align_factor;
        if (front_min_ < CAUTION_DIST)
            lin *= (front_min_ / CAUTION_DIST);
        lin = std::max(lin, 0.04f);
    } else {
        lin = 0.02f;
    }

    cmd.linear.x  = lin;
    cmd.angular.z = ang;

    
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
        // GPS
        gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
            "/gps/fix", 10,
            [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
                if (msg->status.status < 0) return;
                state_.updateFromGPS(*msg);
                gps_received_ = true;
                // Update goal bearing whenever GPS updates so gap finder is accurate
                double bear = state_.position.bearingTo(planner_.getGoal());
                obstacle_.setGoalBearing(static_cast<float>(bear));
            });

        // IMU
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10,
            [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
                state_.updateFromIMU(*msg);
                imu_received_ = true;
            });

        // Odometry
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                state_.updateSpeedFromOdom(*msg);
            });

        // LiDAR
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                obstacle_.processScan(msg);
                scan_received_ = true;
            });

        // Stereo depth
        depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/stereo/depth", 10,
            [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                obstacle_.processDepth(msg);
            });

        // Command publisher
        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);


        // 20 Hz control loop
        timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&RoverController::controlLoop, this));

        RCLCPP_INFO(this->get_logger(),
            "RoverController started — goal (%.6f, %.6f)", goal_lat, goal_lon);
    }

    void controlLoop()
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "[CTRL] obs=%d front=%.2f state=%d | scan=%d gps=%d imu=%d",
            (int)obstacle_.isObstacle(), obstacle_.getFront(),
            (int)obstacle_.getState(),
            (int)scan_received_, (int)gps_received_, (int)imu_received_);

        if (!gps_received_ || !imu_received_ || !scan_received_)
            return;

        double dist = state_.position.distanceTo(planner_.getGoal());
        if (dist < 0.5) {
            RCLCPP_INFO(this->get_logger(), "[CTRL] Goal reached — STOP");
            geometry_msgs::msg::Twist stop;
            stop.linear.x  = 0.0;
            stop.angular.z = 0.0;
            cmd_pub_->publish(stop);
            return;
        }

        if (planner_.isGoalReached()) {
            geometry_msgs::msg::Twist stop;
            cmd_pub_->publish(stop);
            return;
        }

        if (!rclcpp::ok()) return;

        geometry_msgs::msg::Twist cmd;

        if (obstacle_.isObstacle()) {
            cmd = obstacle_.computeAvoidance();
        } else {
            cmd = planner_.computeCommand(state_);

            double bear_fresh = state_.position.bearingTo(planner_.getGoal());
            double rel_bear   = GlobalPlanner::normaliseAngle(bear_fresh - state_.heading_deg + 180.0);
                                obstacle_.setGoalBearing(static_cast<float>(rel_bear));

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "[NAV] dist=%.2fm bear=%.1f° head=%.1f° err=%.1f° lin=%.2f ang=%.2f",
                dist, bear_fresh, state_.heading_deg, rel_bear,
                cmd.linear.x, cmd.angular.z);
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