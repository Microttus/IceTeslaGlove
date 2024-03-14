//
// Created by biomech on 14.03.24.
//

#ifndef ICE_TESLA_GLOVE_SRC_FILE_IMPORTER_H_
#define ICE_TESLA_GLOVE_SRC_FILE_IMPORTER_H_

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "../../src/struct_lib.cc"

class FileImport
{
 public:
  FileImport();

  void import_hand_profiles(const std::string& name_of_profile, HandDouble& MinStruct, HandDouble& MaxStruct);

  void import_operator_glove_pos_settings(HandDouble& MinStruct, HandDouble& MaxStruct);

  std::string tools_directory;
};

#endif //ICE_TESLA_GLOVE_SRC_FILE_IMPORTER_H_
