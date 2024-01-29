//
// Created by biomech on 29.01.24.
//

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

struct FingerForceMeasured{
  double thumb;
  double index;
  double middle;
  double ring;
  double little;
  double palm;
};

struct IceGloveServoPos{
  int thumb;
  int index;
  int middle;
  int ring;
  int little;
  int palm;
};

class IceTeslaGlove : public rclcpp::Node
{
 public:
  IceTeslaGlove()
      : Node("ice_tesla_glove")
      , RobotHandForce{0,0,0,0,0,0}
      , ServoPosGloveOne{0,0,0,0,0,0}
  {
    pub_servo_glove_pos_ = this->create_publisher<geometry_msgs::msg::Twist>("ice_glove_id1", 10);
    timer_ = this->create_wall_timer(
        10ms, std::bind(&IceTeslaGlove::timer_callback, this));

    robot_force_sub_1_ = this->create_subscription<geometry_msgs::msg::Twist>("/robot_force_id1", 10, std::bind(&IceTeslaGlove::update_force_feedback, this, std::placeholders::_1));
  }

 private:
  void timer_callback()
  {
    calculate_servo_pos();
    update_servo_pos();
  }

  void calculate_servo_pos()
  {
    // Read normalized servo value
    // convert normalized value to servo angle
    // add force feedback
    // write to servo struct
  }

  void update_force_feedback(geometry_msgs::msg::Twist::SharedPtr msg)
  {
    RobotHandForce.thumb = msg->linear.x;
    RobotHandForce.index = msg->linear.y;
    RobotHandForce.middle = msg->linear.z;
    RobotHandForce.ring = msg->angular.x;
    RobotHandForce.little = msg->angular.y;
    RobotHandForce.palm = msg->angular.z;
  }

  void update_servo_pos()
  {
    auto servo_msg = geometry_msgs::msg::Twist();

    servo_msg.linear.x = ServoPosGloveOne.thumb;
    servo_msg.linear.y = ServoPosGloveOne.index;
    servo_msg.linear.z = ServoPosGloveOne.middle;
    servo_msg.angular.x = ServoPosGloveOne.ring;
    servo_msg.angular.y = ServoPosGloveOne.little;
    servo_msg.angular.z = ServoPosGloveOne.palm;

    pub_servo_glove_pos_->publish(servo_msg);
  }

  FingerForceMeasured RobotHandForce;
  IceGloveServoPos ServoPosGloveOne;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_servo_glove_pos_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr robot_force_sub_1_;
  //size_t count_{};
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IceTeslaGlove>());
  rclcpp::shutdown();
  return 0;
}