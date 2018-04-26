#include "./sergio_hardware_interface.h"

namespace sergio_control
{

SergioHardwareInterface::SergioHardwareInterface(const std::string& interface_name,
                                                 const std::string& urdf_string,
                                                 const EthercatActuatorDescriptionMap& ethercat_actuators_description) :
  EthercatHardwareInterface(interface_name, urdf_string, ethercat_actuators_description,
                            "sergio_control", "sergio_hardware_interface_node")
{

}

}  // namespace sergio_control
