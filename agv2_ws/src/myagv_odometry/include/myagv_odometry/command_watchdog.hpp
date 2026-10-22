#pragma once

#include <array>
#include <chrono>
#include <cmath>
#include <mutex>

// Independent of ROS time: clock synchronisation must not extend a command lease.
class CommandWatchdog
{
public:
  using Clock = std::chrono::steady_clock;
  void set(double x, double y, double yaw, Clock::time_point now = Clock::now())
  {
    std::lock_guard<std::mutex> lock(mutex_);
    velocity_ = (std::isfinite(x) && std::isfinite(y) && std::isfinite(yaw))
      ? std::array<double, 3>{{x, y, yaw}} : std::array<double, 3>{{0, 0, 0}};
    received_ = now;
    valid_ = true;
  }

  std::array<double, 3> get(double timeout, Clock::time_point now = Clock::now())
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const double age = std::chrono::duration<double>(now - received_).count();
    if (!valid_ || !std::isfinite(timeout) || timeout <= 0 || age < 0 || age >= timeout) {
      return {{0, 0, 0}};
    }
    return velocity_;
  }

private:
  std::mutex mutex_;
  std::array<double, 3> velocity_{{0, 0, 0}};
  Clock::time_point received_{};
  bool valid_{false};
};
