// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: logger.hpp
// Author(s): Ayesha Musarrat
//
// Minimal thread-safe logger writing human readable entries to a text file

#pragma once
#include <fstream>
#include <mutex>
#include <string>

namespace bm {

class Logger {
public:
  explicit Logger(const std::string& path);
  ~Logger();
  void write(const std::string& line);

private:
  std::ofstream file_;
  std::mutex mtx_;
};

std::string iso8601_now();

} // namespace bm
