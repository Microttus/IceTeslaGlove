//
// Created by biomech on 29.01.24.
// Source code for ROS2 control of the IceCube Tesla Glove
// Maintainer: Martin Økter
// Area: UiA Grimstad
//

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

struct HandDouble{
  double thumb;
  double index;
  double middle;
  double ring;
  double little;
  double palm;
};

struct HandServoPos{
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
      : Node("ice_tesla_glove_controller")
      , RobotFingerForce{0, 0, 0, 0, 0, 0}
      , OperatorFingerPos{0,0,0,0,0,0}
      , ServoMultiplierNormToPos{0,0,0,0,0,0}
      , ForceFeedbackNormToPos{0,0,0,0,0,0}
      , ServoPosGloveOne{0,0,0,0,0,0}
  {
    pub_servo_glove_pos_ = this->create_publisher<geometry_msgs::msg::Twist>("/ice_glove_id1", 10);
    timer_ = this->create_wall_timer(
        10ms, std::bind(&IceTeslaGlove::timer_callback, this));

    robot_force_sub_1_ = this->create_subscription<geometry_msgs::msg::Twist>("/robot_force_id1", 10, std::bind(&IceTeslaGlove::update_force_feedback, this, std::placeholders::_1));
    operator_pos_sub_1_ = this->create_subscription<geometry_msgs::msg::Twist>("/operator_pos_id1", 10, std::bind(&IceTeslaGlove::update_hand_pos_feedback, this, std::placeholders::_1));
  }

 private:
  void timer_callback()
  {
    calculate_servo_pos();
    update_servo_pos();
  }

  void calculate_servo_pos()
  {
    // Read normalized stretch value.........
    // convert normalized value to servo angle
    // add force feedback
    // write to servo struct

    // ServoPos = (OperatorFingerPos * Multiplier) - (RobotFingerForce * Multiplier)

    ServoPosGloveOne.thumb = static_cast<int>((OperatorFingerPos.thumb*ServoMultiplierNormToPos.thumb)-(RobotFingerForce.thumb*ForceFeedbackNormToPos.thumb));
    ServoPosGloveOne.index = static_cast<int>((OperatorFingerPos.index*ServoMultiplierNormToPos.index)-(RobotFingerForce.index*ForceFeedbackNormToPos.index));
    ServoPosGloveOne.middle = static_cast<int>((OperatorFingerPos.middle*ServoMultiplierNormToPos.middle)-(RobotFingerForce.middle*ForceFeedbackNormToPos.middle));
    ServoPosGloveOne.ring = static_cast<int>((OperatorFingerPos.ring*ServoMultiplierNormToPos.ring)-(RobotFingerForce.ring*ForceFeedbackNormToPos.ring));
    ServoPosGloveOne.little = static_cast<int>((OperatorFingerPos.little*ServoMultiplierNormToPos.little)-(RobotFingerForce.little*ForceFeedbackNormToPos.little));
    ServoPosGloveOne.palm = static_cast<int>((OperatorFingerPos.palm*ServoMultiplierNormToPos.palm)-(RobotFingerForce.palm*ForceFeedbackNormToPos.palm));
  }

  // Read and updates the force exerted ion the fingertip of the robot and
  void update_force_feedback(geometry_msgs::msg::Twist::SharedPtr msg)
  {
    RobotFingerForce.thumb = msg->linear.x;
    RobotFingerForce.index = msg->linear.y;
    RobotFingerForce.middle = msg->linear.z;
    RobotFingerForce.ring = msg->angular.x;
    RobotFingerForce.little = msg->angular.y;
    RobotFingerForce.palm = msg->angular.z;
  }

  // Read and updates the pos feedback from the load cell of the operator glove
  void update_hand_pos_feedback(geometry_msgs::msg::Twist::SharedPtr msg)
  {
    OperatorFingerPos.thumb = msg->linear.x;
    OperatorFingerPos.index = msg->linear.y;
    OperatorFingerPos.middle = msg->linear.z;
    OperatorFingerPos.ring = msg->angular.x;
    OperatorFingerPos.little = msg->angular.y;
    OperatorFingerPos.palm = msg->angular.z;
  }

  // Write the updated servo position values to the glove topic
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

  HandDouble RobotFingerForce;
  HandDouble OperatorFingerPos;
  HandDouble ServoMultiplierNormToPos;
  HandDouble ForceFeedbackNormToPos;

  HandServoPos ServoPosGloveOne;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_servo_glove_pos_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr robot_force_sub_1_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr operator_pos_sub_1_;
  //size_t count_{};
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IceTeslaGlove>());
  rclcpp::shutdown();
  return 0;
}