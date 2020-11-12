/**
 * @file timer.hpp
 * @author Licheng Wen (wenlc@zju.edu.cn)
 * @brief Timer class
 * @date 2020-11-12
 *
 * @copyright Copyright (c) 2020
 *
 */
#pragma once

#include <chrono>
#include <iostream>
/**
 * @brief class for counting time (microsecond)
 *
 */
class Timer {
 public:
  Timer()
      : start_(std::chrono::high_resolution_clock::now()),
        end_(std::chrono::high_resolution_clock::now()) {}

  void reset() { start_ = std::chrono::high_resolution_clock::now(); }

  void stop() { end_ = std::chrono::high_resolution_clock::now(); }

  double elapsedSeconds() const {
    auto timeSpan = std::chrono::duration_cast<std::chrono::duration<double>>(
        end_ - start_);
    return timeSpan.count();
  }

 private:
  std::chrono::high_resolution_clock::time_point start_;
  std::chrono::high_resolution_clock::time_point end_;
};
