#include "myagv_odometry/command_watchdog.hpp"
#include <limits>
#include <stdexcept>

void expect(bool condition)
{
  if (!condition) { throw std::runtime_error("command watchdog assertion failed"); }
}

int main()
{
  CommandWatchdog commands;
  const auto t = CommandWatchdog::Clock::now();
  const std::array<double, 3> zero{{0, 0, 0}};
  expect(commands.get(0.3, t) == zero);
  commands.set(0.1, 0, -0.2, t);
  expect(commands.get(0.3, t)[0] == 0.1);
  expect(commands.get(0.3, t + std::chrono::milliseconds(300)) == zero);
  expect(commands.get(0.3, t - std::chrono::milliseconds(1)) == zero);
  commands.set(std::numeric_limits<double>::quiet_NaN(), 0, 1, t);
  expect(commands.get(0.3, t) == zero);
  commands.set(0.1, 0, -0.2, t);
  expect(commands.get(0, t) == zero);
  commands.set(0, 0, 0, t);
  expect(commands.get(0.3, t) == zero);
}
