#include <chrono>
#include <cinttypes>
#include <memory>
# include <stdlib.h>
#include "tutorial_interfaces/srv/turtle_cmd_mode.hpp"
#include "rclcpp/rclcpp.hpp"

using TurtleCmdMode = tutorial_interfaces::srv::TurtleCmdMode;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("turtlesim_controller_client");
  auto client = node->create_client<TurtleCmdMode>("change_turtle_control_mode");
  while (!client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "client interrupted while waiting for service to appear.");
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "waiting for service to appear...");
  }
  auto request = std::make_shared<TurtleCmdMode::Request>();
  if (argc < 2) {
    request->mode = 1;
  } else {
    request->mode = static_cast<int8_t>(atoi(argv[1]));
    RCLCPP_INFO(node->get_logger(), "Receive control mode cmd %d", request->mode);
  }

  auto result_future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, result_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "service call failed");
    return 1;
  }
  auto result = result_future.get();
  RCLCPP_INFO(
    node->get_logger(), "set control mode: %d fd_mode: %d", request->mode, result->fd_mode);

  rclcpp::shutdown();
  return 0;
}
