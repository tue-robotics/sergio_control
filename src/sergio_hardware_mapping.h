#ifndef SERGIO_HARDWARE_MAPPING_H
#define SERGIO_HARDWARE_MAPPING_H

#include <vector>
#include <string>
#include <iostream>

const size_t ENCODER_COUNTS_PER_CYCLE = 256;

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

struct Actuator
{
  Data data_;

  // TODO: Add pointer to ethercat iface

  Actuator(const std::string name, size_t input_slave, size_t input_channel,
           size_t output_slave, size_t output_channel) : data_(name)
  {
    std::cout << "Actuator " << name << " " << input_slave << " " << input_channel << output_slave << " " << output_channel << std::endl;
  }
};

#endif
