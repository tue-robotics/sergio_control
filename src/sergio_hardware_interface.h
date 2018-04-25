//
// Copyright (c) 2018 TUe Robotics
//
// @author Rein Appeldoorn (reinzor)
//

#pragma once

#include "./ethercat_hardware_interface.h"

namespace sergio_control
{

using namespace ethercat_hardware_interface;

class SergioHardwareInterface : public EthercatHardwareInterface
{
public:
  SergioHardwareInterface(const std::string& interface_name, const std::string& urdf_string,
                          const EthercatActuatorDescriptionMap& ethercat_actuators_description);
};
}  // namespace sergio_control
