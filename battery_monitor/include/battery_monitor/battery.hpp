// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: battery.hpp
// Author(s): Ayesha Musarrat
//
// Description:
// Battery model for virtual battery simulation. Supports time-based drain,
// optional velocity-based drain (via /odom), low/critical detection, and
// simple getters/setters for integration.

#pragma once
#include <algorithm>
#include <chrono>

//dock_request for navigation node to subcscribe to
//#include "std_msgs/msg/bool.hpp"
//rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr dock_pub_;

namespace bm {

struct BatteryParams {
  float start_percent = 100.0f;     // initial SOC
  float base_drain_per_sec = 0.05f; // %/s baseline drain (idle)
  float velocity_drain_gain = 0.02f;// additional %/s per m/s (optional/not implemented yet)
  float low_threshold = 20.0f;      // % "low"
  float critical_threshold = 10.0f; // % "critical"
};

class Battery {
public:
  explicit Battery(const BatteryParams& p): params_(p), level_(p.start_percent) {}

  // Advance simulation by dt seconds. Optionally provide linear speed (m/s).
  void step(float dt_sec, float linear_speed_mps = 0.0f) {
    if (level_ <= 0.0f) return;
    float drain = params_.base_drain_per_sec * dt_sec;
    drain += std::max(0.0f, linear_speed_mps) * params_.velocity_drain_gain * dt_sec;
    level_ = std::max(0.0f, level_ - drain);
  }

  float percent() const { return level_; }
  bool isLow() const { return level_ <= params_.low_threshold; }
  bool isCritical() const { return level_ <= params_.critical_threshold; }

  void setPercent(float p) { level_ = std::clamp(p, 0.0f, 100.0f); }

private:
  BatteryParams params_;
  float level_;
};

} // namespace bm
