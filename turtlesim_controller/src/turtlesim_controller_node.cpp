#include "turtlesim_controller/turtlesim_controller_node.hpp"

using namespace std::chrono_literals;


/*
获取系统启动后的运行时间(微秒)
*/
inline static uint64_t getSystemTimestampUS()
{
  struct timespec ts;
  // 该函数是用于获取特定 时钟的时间，常用如下4种时钟
  /*
  CLOCK_REALTIME                  //系统当前时间，从1970年1.1日算起
  CLOCK_MONOTONIC                 //系统的启动后运行时间，不能被设置
  CLOCK_PROCESS_CPUTIME_ID        //本进程运行时间
  CLOCK_THREAD_CPUTIME_ID         //本线程运行时间
  */
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)(ts.tv_sec * 1e6 + ts.tv_nsec * 0.001);
}

/*
获取系统启动后的运行时间(毫秒)
*/
static uint64_t getSystemTimestampMS()
{
  return getSystemTimestampUS() / 1000;
}


TurtlesimController::TurtlesimController()
: Node("turtlesim_controller")
{
  //declare params  需要声明参数，不声明编译不过
  declare_parameter("execute_frequency", 30);
  declare_parameter("walk_distance", 3.0);
  declare_parameter("print_execute_duration", true);
  declare_parameter("show_str", "hello world0");


  //init params
  execute_frequency_ = get_parameter("execute_frequency").as_int();
  int execute_interval_ms = 1000 / execute_frequency_;
  walk_distance_ = get_parameter("walk_distance").as_double();
  print_execute_duration_ = get_parameter("print_execute_duration").as_bool();
  show_str_ = get_parameter("show_str").as_string();

  RCLCPP_INFO(
    this->get_logger(), "execute_interval_ms: %d execute_frequency: %d \
    Hz walk_distance: %.3f \
    print_execute_duration: %d show_str: %s",
    execute_interval_ms, execute_frequency_, walk_distance_, print_execute_duration_,
    show_str_.c_str());


  // Setup callback for changes to parameters.
  parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
    this->get_node_base_interface(),
    this->get_node_topics_interface(),
    this->get_node_graph_interface(),
    this->get_node_services_interface());

  parameter_event_sub_ = parameters_client_->on_parameter_event(
    std::bind(&TurtlesimController::on_parameter_event_callback, this, std::placeholders::_1));


  vel_cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 1);
  turtlesim_pose_subscription_ = this->create_subscription<turtlesim::msg::Pose>(
    "turtle1/pose", 1,
    std::bind(&TurtlesimController::turtlesim_pose_callback, this, std::placeholders::_1));

  service_server_ = this->create_service<TurtleCmdMode>(
    "change_turtle_control_mode",
    std::bind(
      &TurtlesimController::handle_turtle_control_mode_service, this,
      std::placeholders::_1, std::placeholders::_2));

  action_server_ = rclcpp_action::create_server<GoLine>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "turtle_go_line",
    std::bind(
      &TurtlesimController::handle_goal, this, std::placeholders::_1,
      std::placeholders::_2),
    std::bind(&TurtlesimController::handle_cancel, this, std::placeholders::_1),
    std::bind(&TurtlesimController::handle_accepted, this, std::placeholders::_1));


  std::chrono::milliseconds interval(execute_interval_ms);
  timer_ = this->create_wall_timer(
    interval, std::bind(&TurtlesimController::timer_callback, this));

}


rclcpp_action::GoalResponse TurtlesimController::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const GoLine::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal {%f, %f}", goal->goal_x, goal->goal_y);
  (void)uuid;
  // Let's reject sequences if we close to goal
  if (has_reached_point(goal->goal_x, goal->goal_y)) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  receive_action_goal_ = true;
  goal_.theta = fmod(
    atan2(
      goal->goal_y - turtlesim_pose_->y,
      goal->goal_x - turtlesim_pose_->x), 2.0f * static_cast<float>(PI));
  // wrap goal_.theta to [-pi, pi)
  if (goal_.theta >= static_cast<float>(PI)) {goal_.theta -= 2.0f * static_cast<float>(PI);}
  RCLCPP_INFO(
    this->get_logger(), "current theta %f goal theta %f", turtlesim_pose_->theta, goal_.theta);
  if (fabsl(goal_.theta - turtlesim_pose_->theta) < 0.01) {
    goal_.x = goal->goal_x;
    goal_.y = goal->goal_y;
    goal_.theta = turtlesim_pose_->theta;
    cmd_mode_ = FORWARD;
  } else {
    //set rotate angle
    goal_.x = turtlesim_pose_->x;
    goal_.y = turtlesim_pose_->y;
    cmd_mode_ = ROTATE;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TurtlesimController::handle_cancel(
  const std::shared_ptr<GoalHandleGoLine> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void TurtlesimController::execute(const std::shared_ptr<GoalHandleGoLine> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing goal");
  rclcpp::Rate loop_rate(5);
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<GoLine::Feedback>();
  feedback->current_pose_x = turtlesim_pose_->x;
  feedback->current_pose_y = turtlesim_pose_->y;
  feedback->current_pose_theta = turtlesim_pose_->theta;

  auto result = std::make_shared<GoLine::Result>();

  while (rclcpp::ok()) {
    // Check if there is a cancel request
    if (goal_handle->is_canceling()) {
      result->str = "Goal Canceled";
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal Canceled");
      return;
    }

    if (cmd_mode_ == ROTATE) {
      if (has_reached_goal()) {
        goal_.x = goal->goal_x;
        goal_.y = goal->goal_y;
        goal_.theta = turtlesim_pose_->theta;
        cmd_mode_ = FORWARD;
      } else {
        rotate();
      }
    } else if (cmd_mode_ == FORWARD) {
      if (has_reached_point(goal->goal_x, goal->goal_y)) {// Check if goal is done
        result->str = "Goal Succeeded";
        receive_action_goal_ = false;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
        stop_turtle();
        break;
      } else {
        move_forward();
      }
    }

    feedback->current_pose_x = turtlesim_pose_->x;
    feedback->current_pose_y = turtlesim_pose_->y;
    feedback->current_pose_theta = turtlesim_pose_->theta;
    // Publish feedback
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(this->get_logger(), "Publish Feedback");
    loop_rate.sleep();
  }
}

void TurtlesimController::handle_accepted(const std::shared_ptr<GoalHandleGoLine> goal_handle)
{
  using namespace std::placeholders;
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&TurtlesimController::execute, this, _1), goal_handle}.detach();
}


void TurtlesimController::timer_callback()
{
  static uint64_t last_time;
  uint64_t now_time = getSystemTimestampMS();
  if (print_execute_duration_) {
    RCLCPP_INFO(this->get_logger(), "execute duration %d ms", now_time - last_time);
  }

  if (receive_action_goal_) {
    last_time = now_time;
    return;
  }

  if (has_reached_goal() && cmd_mode_ != STOP) {
    cmd_mode_ = STOP;
    stop_turtle();
    RCLCPP_INFO(this->get_logger(), "finish task and switch to stop state.");
    return;
  }

  switch (cmd_mode_) {
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
  last_time = now_time;
}

void TurtlesimController::turtlesim_pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
{
  turtlesim_pose_ = msg;
  // RCLCPP_INFO(this->get_logger(), "got turtlesim pose {%f, %f, %f}", msg->x, msg->y, msg->theta);

}

void TurtlesimController::handle_turtle_control_mode_service(
  const std::shared_ptr<TurtleCmdMode::Request> request,
  std::shared_ptr<TurtleCmdMode::Response> response)//这两种写法都可以
// void handle_turtle_control_mode_service(
//   const tutorial_interfaces::srv::TurtleCmdMode::Request::SharedPtr request,
//   tutorial_interfaces::srv::TurtleCmdMode::Response::SharedPtr response)//这两种写法都可以
{
  if (request->mode >= MAX_CONTROL_MODE) {
    RCLCPP_WARN(this->get_logger(), "control mode is out of range.mode is %d", request->mode);
    return;
  }

  if (request->mode == FORWARD) {
    goal_.x = cos(turtlesim_pose_->theta) * walk_distance_ + turtlesim_pose_->x;
    goal_.y = sin(turtlesim_pose_->theta) * walk_distance_ + turtlesim_pose_->y;
    goal_.theta = turtlesim_pose_->theta;
    cmd_mode_ = FORWARD;
  } else if (request->mode == ROTATE) {//向左旋转90度
    goal_.x = turtlesim_pose_->x;
    goal_.y = turtlesim_pose_->y;
    goal_.theta =
      fmod(turtlesim_pose_->theta + static_cast<float>(PI) / 2.0f, 2.0f * static_cast<float>(PI));
    // wrap goal_.theta to [-pi, pi)
    if (goal_.theta >= static_cast<float>(PI)) {goal_.theta -= 2.0f * static_cast<float>(PI);}
    cmd_mode_ = ROTATE;
  } else {
    cmd_mode_ = STOP;
    stop_turtle();
  }

  response->fd_mode = request->mode;
}

//判断是否到达目标点
bool TurtlesimController::has_reached_goal()
{
  return fabsf(turtlesim_pose_->x - goal_.x) < 0.1 && fabsf(turtlesim_pose_->y - goal_.y) < 0.1 &&
         fabsf(turtlesim_pose_->theta - goal_.theta) < 0.05;
}

//判断是否到达特定点
bool TurtlesimController::has_reached_point(double x, double y)
{
  return fabsf(turtlesim_pose_->x - x) < 0.1 && fabsf(turtlesim_pose_->y - y) < 0.1;
}

//linear -> x轴线速度
//angular -> 角速度
void TurtlesimController::control_turtle(float linear, float angular)
{
  geometry_msgs::msg::Twist twist;
  twist.linear.x = linear;
  twist.angular.z = angular;
  vel_cmd_publisher_->publish(twist);
}

void TurtlesimController::stop_turtle(void)
{
  control_turtle(0, 0);
}

void TurtlesimController::move_forward(void)
{
  control_turtle(1.0f, 0);
}

void TurtlesimController::rotate(void)
{
  control_turtle(0, 0.4f);
}


void TurtlesimController::on_parameter_event_callback(
  const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
{
  std::lock_guard<std::mutex> l(config_mutex_);

  for (auto & changed_parameter : event->changed_parameters) {
    const auto & type = changed_parameter.value.type;
    const auto & name = changed_parameter.name;
    const auto & value = changed_parameter.value;

    if (type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE) {
      // Trajectory
      if (name == "walk_distance") {
        walk_distance_ = value.double_value;
        RCLCPP_INFO(this->get_logger(), "update walk_distance: %.3f ", walk_distance_);
      }
    } else if (type == rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER) {
      if (name == "execute_frequency") {
        execute_frequency_ = value.integer_value;//不会改变定时器执行频率，仅作为演示用
        RCLCPP_INFO(this->get_logger(), "update execute_frequency: %d ", execute_frequency_);
      }
    } else if (type == rcl_interfaces::msg::ParameterType::PARAMETER_BOOL) {
      if (name == "print_execute_duration") {
        print_execute_duration_ = value.bool_value;
        RCLCPP_INFO(
          this->get_logger(), "update print_execute_duration: %d ", print_execute_duration_);
      }
    } else if (type == rcl_interfaces::msg::ParameterType::PARAMETER_STRING) {
      if (name == "show_str") {
        show_str_ = value.string_value;
        RCLCPP_INFO(this->get_logger(), "update show_str: %s ", show_str_.c_str());
      }
    }
  }
}
