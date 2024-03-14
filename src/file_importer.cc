//
// Created by biomech on 14.03.24.
//

#include "../include/ice_tesla_glove/file_importer.h"

FileImport::FileImport()
{
  std::string package_name = "ice_tesla_glove";
  tools_directory = ament_index_cpp::get_package_share_directory(package_name) + "/tools/";
}

void FileImport::import_hand_profiles(const std::string& name_of_profile, HandDouble& MinStruct, HandDouble& MaxStruct) {
  // Open the CSV file
  std::ifstream file(tools_directory + "icecube_tesla_glove_profiles.csv");

  // Check if the file is open
  if (!file.is_open()) {
    std::cerr << "Error opening Operator file." << std::endl;
    std::cerr << tools_directory << std::endl;
    return;
  }
  // Define a 2D vector to store the data
  std::vector<std::vector<std::string>> data;

  // Read each line from the CSV file
  std::string line;
  while (std::getline(file, line)) {
    // Use a string stream to split the line into tokens
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

  // Save Operator hand info
  for (auto &row : data) {
    if (row[0] == name_of_profile) {
      MinStruct.thumb = std::stod(row[2]);
      MaxStruct.thumb = std::stod(row[1]);
      MinStruct.index = std::stod(row[3]);
      MaxStruct.index = std::stod(row[4]);
      MinStruct.middle = std::stod(row[5]);
      MaxStruct.middle = std::stod(row[6]);
      MinStruct.ring = std::stod(row[7]);
      MaxStruct.ring = std::stod(row[8]);
      break;
    }
  }
}

void FileImport::import_operator_glove_pos_settings(HandDouble& MinStruct, HandDouble& MaxStruct){
  // Open the CSV file
  std::ifstream file(tools_directory + "operator_finger_pos_cal.csv");

  // Check if the file is open
  if (!file.is_open()) {
    std::cerr << "Error opening glove pos settings file." << std::endl;
    return;
  }

  // Define a 2D vector to store the data
  std::vector<std::vector<std::string>> data;

  // Read each line from the CSV file
  std::string line;
  while (std::getline(file, line)) {
    // Use a string stream to split the line into tokens
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

  for (auto &row : data) {
    if (row[0] == "thumb") {
      MaxStruct.thumb = std::stod(row[1]);
      MinStruct.thumb = std::stod(row[2]);
    } else if (row[0] == "index") {
      MaxStruct.index = std::stod(row[1]);
      MinStruct.index = std::stod(row[2]);
    } else if (row[0] == "middle") {
      MaxStruct.middle = std::stod(row[1]);
      MinStruct.middle = std::stod(row[2]);
    } else if (row[0] == "ring") {
      MaxStruct.ring = std::stod(row[1]);
      MinStruct.ring = std::stod(row[2]);
    } else if (row[0] == "little") {
      MaxStruct.little = std::stod(row[1]);
      MinStruct.little = std::stod(row[2]);
    } else if (row[0]  == "palm") {
      MaxStruct.palm = std::stod(row[1]);
      MinStruct.palm = std::stod(row[2]);
    } else {
      //RCLCPP_INFO(rclcpp::get_logger("IceTeslaGlove"), "Operator Glove settings have settings not implemented");
      std::cout << "Operator Glove settings have settings not implemented" << std::endl;
    }
  }
}