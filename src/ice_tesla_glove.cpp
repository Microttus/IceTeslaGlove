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
#include <iostream>
#include <fstream>
#include <sstream>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"

#include "../include/ice_tesla_glove/haptic_structs.h"
#include "../include/ice_tesla_glove/file_importer.h"
#include "../include/ice_tesla_glove/ice_glove_math.h"

using namespace std::chrono_literals;

class IceTeslaGlove : public rclcpp::Node
{
 public:
  IceTeslaGlove()
      : Node("ice_tesla_glove_controller")
      , callback_time(10)
      , RobotFingerForce{0, 0, 0, 0, 0, 0}
      , OperatorFingerPos{0,0,0,0,0,0}
      , OperatorFingerPosServo{0,0,0,0,0,0}
      , ForceFeedbackNormToPos{50,50,50,50,0,0}
      , ServoMultiplierNormToPosMin{0,0,0,0,0,0}
      , ServoMultiplierNormToPosMax{0,0,0,0,0,0}
      , OperatorPositionMin{0,0,0,0,0,0}
      , OperatorPositionMax{0,0,0,0,0,0}
      , ServoPosGloveOne{0,0,0,0,0,0}
      , ForceOperator({{1,0,0.1},{1,0,0.1},{1,0,0.1},{1,0,0.1},{1,0,0},{0,0,0}})
      , RightHand({{0, 0, 0, 0, 0, ForceOperator.thumb}, {0,0,0,0, 0, ForceOperator.index}, {0,0,0,0, 0, ForceOperator.middle}, {0,0,0,0, 0, ForceOperator.ring}, {0,0,0,0,0,ForceOperator.little}, {0,0,0,0,0,ForceOperator.palm}})
      , name_of_profile("proto_1")
  {
    rclcpp::QoS micro_ros_qos_profile(rclcpp::KeepLast(10));
    micro_ros_qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    micro_ros_qos_profile.history(rclcpp::HistoryPolicy::KeepLast);
    micro_ros_qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);
    //micro_ros_qos_profile.deadline(std::chrono::seconds(1));
    micro_ros_qos_profile.liveliness(rclcpp::LivelinessPolicy::Automatic);

    pub_servo_glove_pos_ = this->create_publisher<geometry_msgs::msg::Twist>("/ice_glove_id1", 10);
    pub_robot_hand_pos_ = this->create_publisher<std_msgs::msg::Float64>("/robot_hand_set_grip", 10);
    timer_ = this->create_wall_timer(10ms, std::bind(&IceTeslaGlove::timer_callback, this));

    // This one can be changed between use of motor force or fingertip sensors
    robot_force_sub_1_ = this->create_subscription<geometry_msgs::msg::Twist>("/robot_finger_force_id1", 10, std::bind(&IceTeslaGlove::update_force_feedback, this, std::placeholders::_1));
    operator_pos_sub_1_ = this->create_subscription<geometry_msgs::msg::Twist>("/ice_glove_pos_id1", micro_ros_qos_profile, std::bind(&IceTeslaGlove::update_hand_pos_feedback, this, std::placeholders::_1));

    // Use FileImport class for updating the settings
    FileImporter.import_hand_profiles(name_of_profile, ServoMultiplierNormToPosMin, ServoMultiplierNormToPosMax);
    FileImporter.import_operator_glove_pos_settings(OperatorPositionMin, OperatorPositionMax);

    RCLCPP_INFO(rclcpp::get_logger("IceTeslaGlove"), "ice_tesla_clove_controller configured ...");

    std::signal(SIGINT, &IceTeslaGlove::onShutdown);

  }

 private:
  void timer_callback()
  {
    //calculate_servo_pos();              // If not i2c feedforward is used
    //calculate_servo_pos_feedforward();  // Use if for feedforward
    calculate_servo_pos_springs();        // Use if admittance calc feedback are to be used
    update_servo_pos();
    update_robot_hand_pos();
  }

  static void onShutdown(int signum) {
    if (signum == SIGINT) {
      // Perform cleanup operations before shutting down
      RCLCPP_INFO(rclcpp::get_logger("IceTeslaGlove"), "Ice gone, meltdown begun...");
      // Add your cleanup logic here

      // Call the ROS 2 shutdown function
      rclcpp::shutdown();
    }
  }

  void calculate_servo_pos()
  {
    // Read normalized stretch value.........
    // convert normalized value to servo angle
    // add force feedback
    // write to servo struct

    // ServoPos = (OperatorFingerPos * Multiplier) - (RobotFingerForce * Multiplier)
    to_operator_profile_servo_pos();

    ServoPosGloveOne.thumb = static_cast<int>((OperatorFingerPosServo.thumb)-(RobotFingerForce.thumb*ForceFeedbackNormToPos.thumb));
    ServoPosGloveOne.index = static_cast<int>((OperatorFingerPosServo.index)-(RobotFingerForce.index*ForceFeedbackNormToPos.index));
    ServoPosGloveOne.middle = static_cast<int>((OperatorFingerPosServo.middle)-(RobotFingerForce.middle*ForceFeedbackNormToPos.middle));
    ServoPosGloveOne.ring = static_cast<int>((OperatorFingerPosServo.ring)-(RobotFingerForce.ring*ForceFeedbackNormToPos.ring));
    ServoPosGloveOne.little = static_cast<int>((OperatorFingerPosServo.little)-(RobotFingerForce.little*ForceFeedbackNormToPos.little));
    ServoPosGloveOne.palm = static_cast<int>((OperatorFingerPosServo.palm)-(RobotFingerForce.palm*ForceFeedbackNormToPos.palm));
  }

  void calculate_servo_pos_feedforward()
  {
    ServoPosGloveOne.thumb = static_cast<int>((RobotFingerForce.thumb*ForceFeedbackNormToPos.thumb));
    ServoPosGloveOne.index = static_cast<int>((RobotFingerForce.index*ForceFeedbackNormToPos.index));
    ServoPosGloveOne.middle = static_cast<int>((RobotFingerForce.middle*ForceFeedbackNormToPos.middle));
    ServoPosGloveOne.ring = static_cast<int>((RobotFingerForce.ring*ForceFeedbackNormToPos.ring));
    ServoPosGloveOne.little = static_cast<int>((RobotFingerForce.little*ForceFeedbackNormToPos.little));
    ServoPosGloveOne.palm = static_cast<int>((RobotFingerForce.palm*ForceFeedbackNormToPos.palm));
  }

  void calculate_servo_pos_springs()
  {
    ServoPosGloveOne.thumb = static_cast<int>(igm.admittance_calc(RightHand.thumb ,RobotFingerForce.thumb, callback_time));
    ServoPosGloveOne.index = static_cast<int>(igm.admittance_calc(RightHand.index, RobotFingerForce.index, callback_time));
    ServoPosGloveOne.middle = static_cast<int>(igm.admittance_calc(RightHand.middle, RobotFingerForce.middle, callback_time));
    ServoPosGloveOne.ring = static_cast<int>(igm.admittance_calc(RightHand.ring, RobotFingerForce.ring, callback_time));
    ServoPosGloveOne.little = static_cast<int>(igm.admittance_calc(RightHand.little, RobotFingerForce.little, callback_time));
    ServoPosGloveOne.palm = static_cast<int>(igm.admittance_calc(RightHand.palm, RobotFingerForce.palm, callback_time));
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
    OperatorFingerPos.thumb = igm.to_norm_val_with_guard(float(msg->linear.x), OperatorPositionMax.thumb, OperatorPositionMin.thumb, OperatorFingerPos.thumb);
    OperatorFingerPos.index = igm.to_norm_val_with_guard(float(msg->linear.y), OperatorPositionMax.index, OperatorPositionMin.index, OperatorFingerPos.index);
    OperatorFingerPos.middle = igm.to_norm_val_with_guard(float(msg->linear.z), OperatorPositionMax.middle, OperatorPositionMin.middle, OperatorFingerPos.middle);
    OperatorFingerPos.ring = igm.to_norm_val_with_guard(float(msg->angular.x), OperatorPositionMax.ring, OperatorPositionMin.ring, OperatorFingerPos.ring);
    OperatorFingerPos.little = igm.to_norm_val_with_guard(float(msg->angular.y), OperatorPositionMax.little, OperatorPositionMin.little, OperatorFingerPos.little);
    OperatorFingerPos.palm = igm.to_norm_val_with_guard(float(msg->angular.z), OperatorPositionMax.palm, OperatorPositionMin.palm, OperatorFingerPos.palm);
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

  // Calculate a middle value to which is published for control of the robotic gripper
  void update_robot_hand_pos()
  {
    auto robot_hand_msg = std_msgs::msg::Float64();

    //double msg_data = (OperatorFingerPos.thumb + OperatorFingerPos.index + OperatorFingerPos.middle)/3.0;
    double msg_data = OperatorFingerPos.middle;

    robot_hand_msg.data = static_cast<double>(std::abs(msg_data - 1.0));

    pub_robot_hand_pos_->publish(robot_hand_msg);
  }

  // Function to set position values according to profile
  void to_operator_profile_servo_pos(){
    OperatorFingerPosServo.thumb = igm.map_output(OperatorFingerPos.thumb, ServoMultiplierNormToPosMin.thumb, ServoMultiplierNormToPosMax.thumb);
    OperatorFingerPosServo.index = igm.map_output(OperatorFingerPos.index, ServoMultiplierNormToPosMin.index, ServoMultiplierNormToPosMax.index);
    OperatorFingerPosServo.middle = igm.map_output(OperatorFingerPos.middle, ServoMultiplierNormToPosMin.middle, ServoMultiplierNormToPosMax.middle);
    OperatorFingerPosServo.ring = igm.map_output(OperatorFingerPos.ring, ServoMultiplierNormToPosMin.ring, ServoMultiplierNormToPosMax.ring);
    OperatorFingerPosServo.little = igm.map_output(OperatorFingerPos.little, ServoMultiplierNormToPosMin.little, ServoMultiplierNormToPosMax.little);
    OperatorFingerPosServo.palm = igm.map_output(OperatorFingerPos.palm, ServoMultiplierNormToPosMin.palm, ServoMultiplierNormToPosMax.palm);
  }

  int callback_time;

  HandDouble RobotFingerForce;
  HandDouble OperatorFingerPos;
  HandDouble OperatorFingerPosServo;
  HandDouble ForceFeedbackNormToPos;

  HandDouble ServoMultiplierNormToPosMin;
  HandDouble ServoMultiplierNormToPosMax;
  HandDouble OperatorPositionMin;
  HandDouble OperatorPositionMax;

  HandServoPos ServoPosGloveOne;

  HandSpring ForceOperator;
  HandStorage RightHand;

  FileImport FileImporter;
  IceGloveMath igm;


  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_servo_glove_pos_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_robot_hand_pos_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr robot_force_sub_1_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr operator_pos_sub_1_;

  std::string name_of_profile;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node =  std::make_shared<IceTeslaGlove>();

  // Create a SingleThreadedExecutor
  rclcpp::executors::SingleThreadedExecutor executor;

  // Add your node to the executor
  executor.add_node(node);

  // Run the executor
  executor.spin();

  rclcpp::shutdown();
  return 0;
}