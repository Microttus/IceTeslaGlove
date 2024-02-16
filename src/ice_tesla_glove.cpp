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
      , OperatorFingerPosServo{0,0,0,0,0,0}
      , ServoMultiplierNormToPosMin{0,0,0,0,0,0}
      , ServoMultiplierNormToPosMax{0,0,0,0,0,0}
      , ForceFeedbackNormToPos{0,0,0,0,0,0}
      , ServoPosGloveOne{0,0,0,0,0,0}
      , name_of_profile("proto_1")
  {
    rclcpp::QoS micro_ros_qos_profile(rclcpp::KeepLast(10));
    micro_ros_qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    micro_ros_qos_profile.history(rclcpp::HistoryPolicy::KeepLast);
    micro_ros_qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);
    //micro_ros_qos_profile.deadline(std::chrono::seconds(1));
    micro_ros_qos_profile.liveliness(rclcpp::LivelinessPolicy::Automatic);

    pub_servo_glove_pos_ = this->create_publisher<geometry_msgs::msg::Twist>("/ice_glove_id1", 10);
    timer_ = this->create_wall_timer(10ms, std::bind(&IceTeslaGlove::timer_callback, this));

    robot_force_sub_1_ = this->create_subscription<geometry_msgs::msg::Twist>("/robot_force_id1", 10, std::bind(&IceTeslaGlove::update_force_feedback, this, std::placeholders::_1));
    operator_pos_sub_1_ = this->create_subscription<geometry_msgs::msg::Twist>("/ice_glove_pos_id1", micro_ros_qos_profile, std::bind(&IceTeslaGlove::update_hand_pos_feedback, this, std::placeholders::_1));

    import_hand_profiles();
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
    to_operator_profile_servo_pos();

    ServoPosGloveOne.thumb = static_cast<int>((OperatorFingerPosServo.thumb)-(RobotFingerForce.thumb*ForceFeedbackNormToPos.thumb));
    ServoPosGloveOne.index = static_cast<int>((OperatorFingerPosServo.index)-(RobotFingerForce.index*ForceFeedbackNormToPos.index));
    ServoPosGloveOne.middle = static_cast<int>((OperatorFingerPosServo.middle)-(RobotFingerForce.middle*ForceFeedbackNormToPos.middle));
    ServoPosGloveOne.ring = static_cast<int>((OperatorFingerPosServo.ring)-(RobotFingerForce.ring*ForceFeedbackNormToPos.ring));
    ServoPosGloveOne.little = static_cast<int>((OperatorFingerPosServo.little)-(RobotFingerForce.little*ForceFeedbackNormToPos.little));
    ServoPosGloveOne.palm = static_cast<int>((OperatorFingerPosServo.palm)-(RobotFingerForce.palm*ForceFeedbackNormToPos.palm));
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
    OperatorFingerPos.thumb = to_norm_val_with_guard(float(msg->linear.x), 3700.0f, 1300.0f, OperatorFingerPos.thumb);
    OperatorFingerPos.index = to_norm_val_with_guard(float(msg->linear.y), 3000.0f, 950.0f, OperatorFingerPos.index);
    OperatorFingerPos.middle = to_norm_val_with_guard(float(msg->linear.z), 3200.0f, 1900.f, OperatorFingerPos.middle);
    OperatorFingerPos.ring = to_norm_val_with_guard(float(msg->angular.x), 3000.0f, 1000.0f, OperatorFingerPos.ring);
    OperatorFingerPos.little = to_norm_val_with_guard(float(msg->angular.y), 3000.0f, 1000.0f, OperatorFingerPos.little);
    OperatorFingerPos.palm = to_norm_val_with_guard(float(msg->angular.z), 1.0f, 0.0f, OperatorFingerPos.palm);
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

  // Function to normalize position values with guard for outliers
  static float to_norm_val_with_guard(float input_val, float upper_val, float lower_val, float last_val=0){
      if (input_val > upper_val or input_val < lower_val){
        return last_val;
      } else {
        return (input_val - lower_val) / (upper_val - lower_val);
      }
  }

  double map_output(double in_val, double out_min, double out_max){
    return (in_val - 0.0f) * (out_max - out_min) / (1.0f - 0.0f) + out_min;
  }

  // Function to set position values according to profile
  void to_operator_profile_servo_pos(){
    OperatorFingerPosServo.thumb = map_output(OperatorFingerPos.thumb, ServoMultiplierNormToPosMin.thumb, ServoMultiplierNormToPosMax.thumb);
    OperatorFingerPosServo.index = map_output(OperatorFingerPos.index, ServoMultiplierNormToPosMin.index, ServoMultiplierNormToPosMax.index);
    OperatorFingerPosServo.middle = map_output(OperatorFingerPos.middle, ServoMultiplierNormToPosMin.middle, ServoMultiplierNormToPosMax.middle);
    OperatorFingerPosServo.ring = map_output(OperatorFingerPos.ring, ServoMultiplierNormToPosMin.ring, ServoMultiplierNormToPosMax.ring);
    OperatorFingerPosServo.little = map_output(OperatorFingerPos.little, ServoMultiplierNormToPosMin.little, ServoMultiplierNormToPosMax.little);
    OperatorFingerPosServo.palm = map_output(OperatorFingerPos.palm, ServoMultiplierNormToPosMin.palm, ServoMultiplierNormToPosMax.palm);
  }

  void import_hand_profiles(){
    // Open the CSV file
    std::ifstream file("icecube_tesla_glove_profiles.csv");

    // Check if the file is open
    if (!file.is_open()) {
      std::cerr << "Error opening file." << std::endl;
      return;
    }

    // Define a 2D vector to store the data
    std::vector<std::vector<std::string>> data;

    // Read each line from the CSV file
    std::string line;
    while (std::getline(file, line)) {
      // Use a stringstream to split the line into tokens
      std::istringstream iss(line);
      std::vector<std::string> tokens;

      // Read each token separated by commas
      std::string token;
      while (std::getline(iss, token, ',')) {
        tokens.push_back(token);
      }

      // Add the tokens to the data vector
      data.push_back(tokens);
    }

    // Close the file
    file.close();

    // Display the read data
    for (auto& row : data) {
      if (row[0] == name_of_profile) {
        ServoMultiplierNormToPosMin.thumb = std::stod(row[2]);
        ServoMultiplierNormToPosMax.thumb = std::stod(row[1]);
        ServoMultiplierNormToPosMin.index = std::stod(row[3]);
        ServoMultiplierNormToPosMax.index = std::stod(row[4]);
        ServoMultiplierNormToPosMin.middle = std::stod(row[5]);
        ServoMultiplierNormToPosMax.middle = std::stod(row[6]);
        ServoMultiplierNormToPosMin.ring = std::stod(row[7]);
        ServoMultiplierNormToPosMax.ring = std::stod(row[8]);

      }
    }
  }

  HandDouble RobotFingerForce;
  HandDouble OperatorFingerPos;
  HandDouble OperatorFingerPosServo;
  HandDouble ServoMultiplierNormToPosMin;
  HandDouble ServoMultiplierNormToPosMax;
  HandDouble ForceFeedbackNormToPos;

  HandServoPos ServoPosGloveOne;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_servo_glove_pos_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr robot_force_sub_1_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr operator_pos_sub_1_;
  //size_t count_{};
  std::string name_of_profile;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IceTeslaGlove>());
  rclcpp::shutdown();
  return 0;
}