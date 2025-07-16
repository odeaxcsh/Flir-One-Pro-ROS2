#include <csignal>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "driver_flir.hpp" // Adjust include if necessary based on your package structure

void sigsegv_handler(int sig)
{
  std::signal(SIGSEGV, SIG_DFL);
  RCLCPP_ERROR(rclcpp::get_logger("camera_flir_node"), "Segmentation fault, stopping camera.");
  rclcpp::shutdown();
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  std::signal(SIGSEGV, sigsegv_handler);

  auto node = rclcpp::Node::make_shared("camera_flir_node");

  driver_flir::DriverFlir dvr(node);
  dvr.setup();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  std::thread spinner([&executor]() {
    executor.spin();
  });

  while (rclcpp::ok() && dvr.ok()){
    dvr.poll();
  }

  rclcpp::shutdown();
  if (spinner.joinable()) {
    spinner.join();
  }
  dvr.shutdown();

  return 0;
}
