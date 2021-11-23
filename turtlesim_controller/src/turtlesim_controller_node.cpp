
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "turtlesim/msg/pose.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <tutorial_interfaces/srv/turtle_cmd_mode.hpp>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

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

public:
  TurtlesimController()
  : Node("turtlesim_controller")
  {
    vel_cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 1);
    turtlesim_pose_subscription_ = this->create_subscription<turtlesim::msg::Pose>(
      "turtle1/pose", 1, std::bind(&TurtlesimController::turtlesim_pose_callback, this, std::placeholders::_1));

    server_ = this->create_service<TurtleCmdMode>("change_turtle_control_mode", 
      std::bind(&TurtlesimController::handle_turtle_control_mode_service, this, std::placeholders::_1, std::placeholders::_2));
    
    timer_ = this->create_wall_timer(
      20ms, std::bind(&TurtlesimController::timer_callback, this));
  }

private:
  void timer_callback()
  {
    if(has_reached_goal() && cmd_mode_ != STOP)
    {
      cmd_mode_ = STOP;
      stop_turtle();
      RCLCPP_INFO(this->get_logger(), "finish task and switch to stop state.");
      return;
    }

    switch (cmd_mode_)
    {
    case FORWARD:
      move_forward();
      break;
    case ROTATE:
      rotate();
      break;
    
    default:
      cmd_mode_ = STOP;
      stop_turtle();
      break;
    }
    
  }

  void turtlesim_pose_callback(const turtlesim::msg::Pose::SharedPtr msg) 
  {
    turtlesim_pose_ = msg;
    RCLCPP_INFO(this->get_logger(), "got turtlesim pose {%f, %f, %f}", msg->x, msg->y, msg->theta);

  }

  void handle_turtle_control_mode_service(
    const std::shared_ptr<TurtleCmdMode::Request> request,
    std::shared_ptr<TurtleCmdMode::Response> response)//这两种写法都可以
  // void handle_turtle_control_mode_service(
  //   const tutorial_interfaces::srv::TurtleCmdMode::Request::SharedPtr request,
  //   tutorial_interfaces::srv::TurtleCmdMode::Response::SharedPtr response)//这两种写法都可以
  {
    if(request->mode >= MAX_CONTROL_MODE)
    {
      RCLCPP_WARN(this->get_logger(), "control mode is out of range.mode is %d", request->mode);
      return;
    }

    if(request->mode == FORWARD)
    {
      goal_.x = cos(turtlesim_pose_->theta) * 2 + turtlesim_pose_->x;
      goal_.y = sin(turtlesim_pose_->theta) * 2 + turtlesim_pose_->y;
      goal_.theta = turtlesim_pose_->theta;
      cmd_mode_ = FORWARD;
    }
    else if(request->mode == ROTATE)//向左旋转90度
    {
      goal_.x = turtlesim_pose_->x;
      goal_.y = turtlesim_pose_->y;
      goal_.theta = fmod(turtlesim_pose_->theta + static_cast<float>(PI) / 2.0f, 2.0f * static_cast<float>(PI));
      // wrap g_goal.theta to [-pi, pi)
      if (goal_.theta >= static_cast<float>(PI)) goal_.theta -= 2.0f * static_cast<float>(PI);
      cmd_mode_ = ROTATE;
    }
    else
    {
      cmd_mode_ = STOP;
      stop_turtle();
    }

    response->fd_mode = request->mode;
  }

  //判断是否到达目标点
  bool has_reached_goal()
  {
    return fabsf(turtlesim_pose_->x - goal_.x) < 0.1 && fabsf(turtlesim_pose_->y - goal_.y) < 0.1 && fabsf(turtlesim_pose_->theta - goal_.theta) < 0.01;
  }

  //linear -> x轴线速度
  //angular -> 角速度
  void control_turtle(float linear, float angular)
  {
    geometry_msgs::msg::Twist twist;
    twist.linear.x = linear;
    twist.angular.z = angular;
    vel_cmd_publisher_->publish(twist);
  }

  void stop_turtle(void)
  {
    control_turtle(0, 0);
  }

  void move_forward(void)
  {
    control_turtle(1.0f, 0);
  }

  void rotate(void)
  {
    control_turtle(0, 0.4f);
  }


  std_msgs::msg::UInt8 tmp;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_cmd_publisher_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtlesim_pose_subscription_;
  rclcpp::Service<tutorial_interfaces::srv::TurtleCmdMode>::SharedPtr server_;

  turtlesim::msg::Pose::SharedPtr turtlesim_pose_;
  turtlesim::msg::Pose goal_;
  ControlMode cmd_mode_ = STOP;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtlesimController>());
  rclcpp::shutdown();
  return 0;
}
