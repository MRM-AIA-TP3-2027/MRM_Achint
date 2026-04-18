// #pragma once

// #include <new>
// #include <cstddef>
// #include <sensor_msgs/msg/laser_scan.hpp>
// #include <sensor_msgs/msg/image.hpp>
// #include <geometry_msgs/msg/twist.hpp>
// #include <rclcpp/rclcpp.hpp>
// #include <algorithm>
// #include <chrono>
// #include <vector>

// // enum class AvoidState { NAVIGATING, AVOIDING, ESCAPING };
// enum class AvoidState { NAVIGATING, AVOIDING };


// class ObstacleAvoidance
// {
// public:
//     // ── Tuning constants ────────────────────────────────────────────────
//     static constexpr float  FRONT_FOV_DEG    = 30.0f;   // ±60° front cone (wide early detection)
//     static constexpr float  CAUTION_DIST     = 2.0f;    // m — start slowing + steering
//     static constexpr float  DANGER_DIST      = 1.0f;    // m — full stop, hard turn
//     static constexpr float  LINEAR_SPEED_MPS = 0.25f;    // max forward speed during avoidance
//     static constexpr float  HARD_TURN_RAD_S  = 1.2f;    // rad/s — max turn rate
//     static constexpr float  MIN_TURN_RAD_S   = 0.8f;    // rad/s — gentle nudge at CAUTION boundary
//     static constexpr double STUCK_TIMEOUT_S  = 5.0;     // seconds before escape triggers
//     static constexpr float CLEAR_DIST = 2.5f;
//      float filtered_angular_ = 0.0f;
//     static constexpr float ANGULAR_SMOOTHING = 0.7f; // 0 = no filter, 1 = heavy
//      // Stuck detection (unchanged)
//     // if (state_ == AvoidState::AVOIDING) {
//     //     double elapsed = std::chrono::duration<double>(now - avoiding_since_).count();
//     //     if (elapsed > STUCK_TIMEOUT_S) {
//     //         state_        = AvoidState::ESCAPING;
//     //         escape_start_ = now;
//     //         escape_phase_ = 0;
//     //         turn_right_   = (right_avg_ > left_avg_);
//     //         RCLCPP_ERROR(rclcpp::get_logger("obstacle_avoidance"),
//     //             "[OBS] LOCAL MINIMA — stuck %.1fs, ESCAPING toward %s",
//     //             elapsed, turn_right_ ? "RIGHT" : "LEFT");
//     //     }
//     // }
   
//     ObstacleAvoidance()
//     : state_(AvoidState::NAVIGATING),
//       front_min_(999.f),
//       left_avg_(999.f),
//       right_avg_(999.f),
//       turn_right_(false),
//       avoiding_since_(std::chrono::steady_clock::now()),
//     //   escape_start_(std::chrono::steady_clock::now()),
//     //   escape_phase_(0),
//       logged_once_(false),
//       angle_min_(0.f),
//       angle_inc_(0.f)
//     {}

//     // ── Public API ──────────────────────────────────────────────────────
//     void processScan(const sensor_msgs::msg::LaserScan::SharedPtr& msg);
//     void processDepth(const sensor_msgs::msg::Image::SharedPtr& msg);
//     void setGoalBearing(double bearing_deg) { goal_bearing_ = bearing_deg; }

//     geometry_msgs::msg::Twist computeAvoidance();

//     bool       isObstacle() const { return state_ != AvoidState::NAVIGATING; }
//     AvoidState getState()   const { return state_;     }
//     float      getFront()   const { return front_min_; }

// private:
//     // ── Sensor state ────────────────────────────────────────────────────
//     std::vector<float> ranges_;
//     float angle_min_;
//     float angle_inc_;

//     float front_min_;   // minimum range in front cone
//     float left_avg_;    // average range +10°..+90°  (left side)
//     float right_avg_;   // average range -10°..-90°  (right side)
//     double goal_bearing_ = 0.0;          // bearing from rover to goal (degrees)
//     float  front_min_angle_ = 0.0f;      // angle of closest front obstacle (degrees)
    

//     // ── FSM state ───────────────────────────────────────────────────────
//     AvoidState state_;
//     bool       turn_right_;    // locked at obstacle detection, re-evaluated in AVOIDING

//     std::chrono::steady_clock::time_point avoiding_since_;
//     // std::chrono::steady_clock::time_point escape_start_;
//     // int  escape_phase_;
//     bool logged_once_;
//     rclcpp::Clock::SharedPtr clock_ =
//     std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);


//     // ── Internal ────────────────────────────────────────────────────────
//     void updateState();

//     float idx2deg(int i) const {
//         return (angle_min_ + i * angle_inc_) * 180.f / static_cast<float>(M_PI);
//     }
// };


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
    static constexpr float CAUTION_DIST      = 1.5f;
    static constexpr float DANGER_DIST       = 0.6f;
    static constexpr float CLEAR_DIST        = 2.0f;
    static constexpr float LINEAR_SPEED_MPS  = 0.15f;  // slow down significantly
    static constexpr float HARD_TURN_RAD_S   = 0.8f;
    static constexpr float  MIN_TURN_RAD_S     = 0.3f;
    static constexpr float  ANGULAR_SMOOTHING  = 0.7f;
    static constexpr float  FRONT_FOV_DEG    = 35.0f;


    // ── Local minima / escape tuning ─────────────────────────────────────
    static constexpr double STUCK_TIMEOUT_S    = 2.0;   // s in AVOIDING before escape
    static constexpr float  STUCK_IMPROVE_M    = 0.05f; // min improvement to reset timer
    static constexpr double ESCAPE_REVERSE_S   = 3.0;   // phase-0 duration (reverse)
    static constexpr double ESCAPE_TURN_S      = 2.5;   // phase-1 duration (hard turn)
    static constexpr float  ESCAPE_REVERSE_MPS = -0.07f;

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

    geometry_msgs::msg::Twist computeAvoidance();

    bool       isObstacle() const { return state_ != AvoidState::NAVIGATING; }
    AvoidState getState()   const { return state_;     }
    float      getFront()   const { return front_min_; }

private:
    // ── Sensor state ────────────────────────────────────────────────────
    std::vector<float> ranges_;
    float angle_min_;
    float angle_inc_;

    float front_min_;
    float left_avg_;
    float right_avg_;
    double goal_bearing_    = 0.0;
    float  front_min_angle_ = 0.0f;
    float gap_angle_ = 0.0f;

    // ── FSM state ───────────────────────────────────────────────────────
    AvoidState state_;
    bool       turn_right_;

    std::chrono::steady_clock::time_point avoiding_since_;
    std::chrono::steady_clock::time_point escape_start_;
    int   escape_phase_;

    // Local-minima detection: track best front distance since AVOIDING began.
    // If front_min_ never improves by STUCK_IMPROVE_M within STUCK_TIMEOUT_S
    // we declare a local minimum and trigger ESCAPING.
    float best_front_while_avoiding_;

    bool logged_once_;

    rclcpp::Clock::SharedPtr clock_ =
        std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);

    // ── Internal ────────────────────────────────────────────────────────
    void updateState();

    float idx2deg(int i) const {
        return (angle_min_ + i * angle_inc_) * 180.f / static_cast<float>(M_PI);
    }
};