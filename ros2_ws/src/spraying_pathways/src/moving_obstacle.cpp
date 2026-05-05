#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class YBounceCmdVel : public rclcpp::Node
{
public:
  YBounceCmdVel() : rclcpp::Node("y_bounce_cmd_vel")
  {
    topic_   = this->declare_parameter<std::string>("topic", "/human_arm/cmd_vel");
    start_y_ = this->declare_parameter<double>("start_y", 3.0);
    end_y_   = this->declare_parameter<double>("end_y", -3.0);
    speed_   = this->declare_parameter<double>("speed", 0.2);
    hz_      = this->declare_parameter<double>("update_rate", 20.0);
    pause_s_ = this->declare_parameter<double>("pause_s", 0.0);

    dist_     = std::abs(end_y_ - start_y_);
    seg_time_ = (speed_ > 1e-6 && dist_ > 1e-6)
                  ? dist_ / speed_
                  : std::numeric_limits<double>::infinity();
    dir_ = (end_y_ >= start_y_) ? 1.0 : -1.0;

    pub_ = this->create_publisher<geometry_msgs::msg::Twist>(topic_, 10);

    const double period = 1.0 / std::max(1.0, hz_);
    move_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(period)),
      std::bind(&YBounceCmdVel::on_tick_, this));

    RCLCPP_INFO(get_logger(), "Bounce: y [%.2f, %.2f] at %.2f m/s on %s",
                std::min(start_y_, end_y_), std::max(start_y_, end_y_),
                speed_, topic_.c_str());
  }

private:
  enum class Phase { Moving, Pause };

  void on_tick_()
  {
    const double dt = 1.0 / std::max(1.0, hz_);
    elapsed_ += dt;

    geometry_msgs::msg::Twist cmd;

    if (phase_ == Phase::Moving) {
      cmd.linear.y = dir_ * speed_;
      if (elapsed_ >= seg_time_) {
        elapsed_ = 0.0;
        if (pause_s_ > 0.0) {
          phase_ = Phase::Pause;
        } else {
          dir_ *= -1.0;
        }
      }
    } else {
      if (elapsed_ >= pause_s_) {
        elapsed_ = 0.0;
        phase_ = Phase::Moving;
        dir_ *= -1.0;
      }
    }

    pub_->publish(cmd);
  }

  std::string topic_;
  double start_y_{}, end_y_{}, speed_{}, hz_{}, pause_s_{};
  double dist_{}, seg_time_{}, dir_{};
  Phase phase_{Phase::Moving};
  double elapsed_{};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr move_timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<YBounceCmdVel>());
  rclcpp::shutdown();
  return 0;
}
