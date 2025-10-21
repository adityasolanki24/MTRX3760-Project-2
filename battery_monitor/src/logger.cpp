/ MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: logger.cpp
// Author(s): Ayesha Musarrat
//
// Implementating Logger and timestamp helper

#include "battery_monitor/logger.hpp"
#include <chrono>
#include <iomanip>
#include <sstream>

namespace bm {

Logger::Logger(const std::string& path) : file_(path, std::ios::app) {
  if (file_.is_open()) {
    file_ << "------------------------------\n";
    file_ << iso8601_now() << "  [BatteryMonitor] Log start\n";
  }
}
Logger::~Logger() {
  if (file_.is_open()) {
    file_ << iso8601_now() << "  [BatteryMonitor] Log end\n";
    file_.close();
  }
}

void Logger::write(const std::string& line) {
  std::lock_guard<std::mutex> lk(mtx_);
  if (file_.is_open()) {
    file_ << iso8601_now() << "  " << line << "\n";
  }
}

std::string iso8601_now() {
  using clock = std::chrono::system_clock;
  auto now = clock::now();
  auto time = clock::to_time_t(now);
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

  std::tm tm{};
  gmtime_r(&time, &tm);
  std::ostringstream oss;
  oss << std::put_time(&tm, "%FT%T") << '.' << std::setw(3) << std::setfill('0') << ms.count() << "Z";
  return oss.str();
}

} // namespace bm
