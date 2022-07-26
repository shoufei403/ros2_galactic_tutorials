#include <chrono>
#include <cinttypes>
#include <functional>
#include <future>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
// TODO(jacobperron): Remove this once it is included as part of 'rclcpp.hpp'
#include "rclcpp_action/rclcpp_action.hpp"
#include <tutorial_interfaces/action/go_line.hpp>

class TurtlesimActionClient : public rclcpp::Node
{
public:
  using GoLine = tutorial_interfaces::action::GoLine;
  using GoalHandleGoLine = rclcpp_action::ClientGoalHandle<GoLine>;

  explicit TurtlesimActionClient(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : Node("turtlesim_action_client", node_options), goal_done_(false)
  {
    this->client_ptr_ = rclcpp_action::create_client<GoLine>(
      this->get_node_base_interface(),
      this->get_node_graph_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "turtle_go_line");

    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&TurtlesimActionClient::send_goal, this));
  }

  bool is_goal_done() const
  {
    return this->goal_done_;
  }

  void send_goal()
  {
    using namespace std::placeholders;

    this->timer_->cancel();


    this->goal_done_ = false;

    if (!this->client_ptr_) {
      RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
    }

    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      this->goal_done_ = true;
      return;
    }

    auto goal_msg = GoLine::Goal();
    goal_msg.goal_x = 5;
    goal_msg.goal_y = 5;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<GoLine>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&TurtlesimActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&TurtlesimActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&TurtlesimActionClient::result_callback, this, _1);
    auto goal_handle_future = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<GoLine>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool goal_done_;

  void goal_response_callback(GoalHandleGoLine::SharedPtr goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandleGoLine::SharedPtr,
    const std::shared_ptr<const GoLine::Feedback> feedback)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "current turtle pose {%f, %f, %f}" PRId32,
      feedback->current_pose_x, feedback->current_pose_y, feedback->current_pose_theta);
  }

  void result_callback(const GoalHandleGoLine::WrappedResult & result)
  {
    this->goal_done_ = true;
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Result received");
    RCLCPP_INFO(this->get_logger(), "%" PRId32, result.result->str);
  }
};  // class TurtlesimActionClient

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto action_client = std::make_shared<TurtlesimActionClient>();

  while (!action_client->is_goal_done()) {
    rclcpp::spin_some(action_client);
  }

  rclcpp::shutdown();
  return 0;
}
