#include "myagv_odometry/myAGV.hpp"
#include <rclcpp/rclcpp.hpp>
#include <thread>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyAGV>();

  if (!node->init()) {
    RCLCPP_ERROR(node->get_logger(), "myAGV initialization failed!");
    return 1;
  }
  RCLCPP_INFO(node->get_logger(), "myAGV initialized successfully.");

  // Spin callbacks (cmd_vel) in a background thread; serial loop runs in main.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });

  rclcpp::Rate loop_rate(20);
  while (rclcpp::ok()) {
    node->execute();
    loop_rate.sleep();
  }

  executor.cancel();
  spin_thread.join();
  rclcpp::shutdown();
  return 0;
}
