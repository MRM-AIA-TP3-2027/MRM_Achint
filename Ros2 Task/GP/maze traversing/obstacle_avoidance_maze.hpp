#pragma once

#include <new>
#include <cstddef>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <chrono>
#include <vector>

enum class AvoidState { NAVIGATING, AVOIDING, ESCAPING };

class ObstacleAvoidance
{
public:
    // ── Tuning constants ────────────────────────────────────────────────
    static constexpr float CAUTION_DIST      = 1.0f;
    static constexpr float SLOW_DIST    = 3.5f; 
    
static constexpr float BRAKE_DIST   =1.0f;
    static constexpr float DANGER_DIST       = 0.8f;
    static constexpr float CLEAR_DIST        = 1.5f;
    static constexpr float LINEAR_SPEED_MPS  = 0.15f;  
    static constexpr float HARD_TURN_RAD_S   = 1.2f;
    static constexpr float  FRONT_FOV_DEG    = 15.0f;
static constexpr float  ESCAPE_GAP_WIDTH_DEG = 40.0f;  // gap must be this wide to drive through

    // ── Local minima / escape tuning ─────────────────────────────────────
    static constexpr double STUCK_TIMEOUT_S    = 4.0;   
    static constexpr float  STUCK_IMPROVE_M    = 0.25f; 
    static constexpr double ESCAPE_REVERSE_S   = 2.5;   
    static constexpr float  ESCAPE_REVERSE_MPS = -0.08f;
    static constexpr double ESCAPE_DRIVE_S = 6.0;
    static constexpr float  SIDE_FREE_THRESH    = 1.0f;  // side clearance = "I'm free"
    static constexpr float  ESCAPE_CLEAR_THRESH = 1.3f;
    
    

    
    float filtered_angular_ = 0.5f;
    int clear_count_ = 0;

    ObstacleAvoidance()
    : state_(AvoidState::NAVIGATING),
      front_min_(999.f),
      left_avg_(999.f),
      right_avg_(999.f),
      turn_right_(false),
      avoiding_since_(std::chrono::steady_clock::now()),
      escape_start_(std::chrono::steady_clock::now()),
      escape_phase_(0),
      best_front_while_avoiding_(999.f),
      logged_once_(false),
      angle_min_(0.f),
      angle_inc_(0.f)
    {}

    // ── Public API ──────────────────────────────────────────────────────
    void processScan(const sensor_msgs::msg::LaserScan::SharedPtr& msg);
    void processDepth(const sensor_msgs::msg::Image::SharedPtr& msg);
    void setGoalBearing(double bearing_deg) { goal_bearing_ = bearing_deg; }
    void suppressGoalPullFor(int scans) {
    suppress_goal_pull_ = true;
    suppress_scan_count_ = scans;
}

    geometry_msgs::msg::Twist computeAvoidance();

    bool       isObstacle() const { return state_ != AvoidState::NAVIGATING; }
    bool getTurnRight() const { return turn_right_; }

    AvoidState getState()   const { return state_;     }
    float      getFront()   const { return front_min_; }
    void forceEscape(int attempt_count) {
    if (state_ == AvoidState::ESCAPING) return;
    state_        = AvoidState::ESCAPING;
    escape_start_ = std::chrono::steady_clock::now();
    escape_phase_ = 0;
    clear_count_  = 0;
    full_escape_attempts_++;

    // Alternate direction every attempt, verified against live scan data
    turn_right_ = (attempt_count % 2 == 0)
                  ? (right_avg_ > left_avg_)
                  : !(right_avg_ > left_avg_);

    suppress_goal_pull_ = false;

    RCLCPP_ERROR(rclcpp::get_logger("obstacle_avoidance"),
        "[FORCE] Escape #%d — turning %s  L=%.2f R=%.2f",
        attempt_count, turn_right_ ? "RIGHT" : "LEFT",
        left_avg_, right_avg_);
}
private:
    //  Sensor state 
    std::vector<float> ranges_;
    float angle_min_;
    float angle_inc_;

    float front_min_;
    float left_avg_;
    float right_avg_;
    float back_left_avg_  = 999.f;  
float back_right_avg_ = 999.f;   
float rear_min_       = 999.f; 
int full_escape_attempts_ = 0;
    bool suppress_goal_pull_ = false;
int  suppress_scan_count_ = 0;  
float gap_width_deg_ = 0.0f;  // width of the best clear sector in degrees


    double goal_bearing_    = 0.0;
    float  front_min_angle_ = 0.0f;
    float gap_angle_ = 0.0f;
    float last_avoid_direction_ = 0.0f;
int   same_dir_count_       = 0;

    AvoidState state_;
    bool       turn_right_;

    std::chrono::steady_clock::time_point avoiding_since_;
    std::chrono::steady_clock::time_point escape_start_;
    int   escape_phase_;

    float best_front_while_avoiding_;

    bool logged_once_;

    rclcpp::Clock::SharedPtr clock_ =
        std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);

    void updateState();

    float idx2deg(int i) const {
        return (angle_min_ + i * angle_inc_) * 180.f / static_cast<float>(M_PI);
    }
};