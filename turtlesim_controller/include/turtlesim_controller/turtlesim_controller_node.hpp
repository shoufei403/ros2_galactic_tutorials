#ifndef TURTLESIM_CONTROLLER__TURTLESIM_CONTROLLER_HPP_
#define TURTLESIM_CONTROLLER__TURTLESIM_CONTROLLER_HPP_


#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cinttypes>
#include <ctime>
#include <sys/time.h>
#include <thread>

#include "rclcpp/rclcpp.hpp"
// TODO(jacobperron): Remove this once it is included as part of 'rclcpp.hpp'
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "turtlesim/msg/pose.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <tutorial_interfaces/srv/turtle_cmd_mode.hpp>
#include <tutorial_interfaces/action/go_line.hpp>



enum ControlMode 
{
  STOP,    //停止
  FORWARD, //直行设定的距离
  ROTATE,  //原地旋转设定的度数
  MAX_CONTROL_MODE,
};

#define PI 3.141592
using TurtleCmdMode = tutorial_interfaces::srv::TurtleCmdMode;


class TurtlesimController : public rclcpp::Node
{
  using GoLine = tutorial_interfaces::action::GoLine;
  using GoalHandleGoLine = rclcpp_action::ServerGoalHandle<GoLine>;

public:
  TurtlesimController();

private:

  void timer_callback();
  void turtlesim_pose_callback(const turtlesim::msg::Pose::SharedPtr msg); 

  void handle_turtle_control_mode_service(
    const std::shared_ptr<TurtleCmdMode::Request> request,
    std::shared_ptr<TurtleCmdMode::Response> response);//这两种写法都可以
  // void handle_turtle_control_mode_service(
  //   const tutorial_interfaces::srv::TurtleCmdMode::Request::SharedPtr request,
  //   tutorial_interfaces::srv::TurtleCmdMode::Response::SharedPtr response);//这两种写法都可以

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const GoLine::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleGoLine> goal_handle);

  void execute(const std::shared_ptr<GoalHandleGoLine> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandleGoLine> goal_handle);


  //判断是否到达目标点
  bool has_reached_goal();
  bool has_reached_point(double x, double y);

  //linear -> x轴线速度
  //angular -> 角速度
  void control_turtle(float linear, float angular);

  void stop_turtle(void);
  void move_forward(void);
  void rotate(void);
  void on_parameter_event_callback(
      const rcl_interfaces::msg::ParameterEvent::SharedPtr event);


  std_msgs::msg::UInt8 tmp;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_cmd_publisher_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtlesim_pose_subscription_;
  rclcpp::Service<tutorial_interfaces::srv::TurtleCmdMode>::SharedPtr service_server_;
  rclcpp_action::Server<tutorial_interfaces::action::GoLine>::SharedPtr action_server_;
  
  turtlesim::msg::Pose::SharedPtr turtlesim_pose_;
  turtlesim::msg::Pose goal_;
  ControlMode cmd_mode_ = STOP;
  bool receive_action_goal_ = false;

  //params
  double walk_distance_ = 2;
  int execute_frequency_ = 0;
  bool print_execute_duration_ = false;
  std::string show_str_ = "";
  

  // Subscription for parameter change
  std::mutex config_mutex_;
  rclcpp::AsyncParametersClient::SharedPtr parameters_client_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;


};











#endif

