
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>

using namespace std::chrono_literals;

using std::placeholders::_1;

class TimeApiTest : public rclcpp::Node
{
public:
  TimeApiTest()
  : Node("time_api_test")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&TimeApiTest::topic_callback, this, _1));

    timer_ = this->create_wall_timer(
      1000ms, std::bind(&TimeApiTest::timer_callback, this));

  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  }

  void timer_callback()
  {

    auto t = rclcpp::Clock().now();
    RCLCPP_INFO(this->get_logger(), "[rclcpp::Clock().now()] sec:%lf nano:%ld", t.seconds(), t.nanoseconds());

    auto t1 = std::chrono::system_clock::now(); 
    time_t tt = std::chrono::system_clock::to_time_t ( t1 );
    RCLCPP_INFO(this->get_logger(), "[std::chrono::system_clock::now()] sec:%ld", tt);

    std::chrono::steady_clock::time_point td = std::chrono::steady_clock::now(); 
    std::chrono::steady_clock::duration dtn = td.time_since_epoch();
    double secs = dtn.count() * std::chrono::steady_clock::period::num / std::chrono::steady_clock::period::den;
    RCLCPP_INFO(this->get_logger(), "[std::chrono::steady_clock::now()] sec:%lf", secs);
    
    auto t2 = this->get_clock()->now();
    RCLCPP_INFO(this->get_logger(), "[get_clock()->now()] sec:%lf nano:%ld", t2.seconds(), t2.nanoseconds());

    auto t3 = this->now();
    RCLCPP_INFO(this->get_logger(), "[this->now()] sec:%lf nano:%ld", t3.seconds(), t3.nanoseconds());

  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TimeApiTest>());
  rclcpp::shutdown();
  return 0;
}
