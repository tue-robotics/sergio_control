#ifndef SERGIO_HARDWARE_MAPPING_H
#define SERGIO_HARDWARE_MAPPING_H

#include <vector>
#include <string>

const std::string ACTUATOR_NAME = "actuator1";
const std::string JOINT_NAME = "joint1";
const std::string JOINT_TO_ACTUATOR_TRANSMISSION_NAME = "joint1_to_actuator1_transmission";
const std::string ACTUATOR_TO_JOINT_TRANSMISSION_NAME = "actuator1_to_joint1_transmission";

struct Data
{
  // state
  double position_ = 0;
  double velocity_ = 0;
  double effort_ = 0;

  // reference
  double command_ = 0;

  std::string name_;

  Data(const std::string name) : name_(name)
  {

  }
};

#endif
