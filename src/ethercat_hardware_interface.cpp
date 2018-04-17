#include "ethercat_hardware_interface.h"

#include <ethercat_interface/exceptions.h>

namespace ethercat_hardware_interface
{
EthercatHardwareInterface::EthercatHardwareInterface(
    const std::string& ethernet_interface, const std::string& urdf_string,
    const std::map<std::string, EthercatActuatorDescription>& ethercat_actuators_description,
    const std::string& package_name, const std::string& executable_name)
  : transmission_manager_(urdf_string)
{
  // 1. Connect to the ethercat interface
  try
  {
    if (!ethercat_interface_.initialize(ethernet_interface))
    {
      throw std::runtime_error("Failed to initialize ethercat interface on ethernet interface " + ethernet_interface);
    }
  }
  catch (SocketError)
  {
    throw std::runtime_error("No socket connection on " + ethernet_interface + ". Try excecuting the following "
                                                                               "command: sudo setcap cap_net_raw+ep "
                                                                               "$(readlink $(catkin_find " +
                             package_name + " " + executable_name + "))\n");
  }

  // 2. Get the actuators based on the description parsed from the URDF
  for (auto actuator_state : transmission_manager_.actuator_states_)
  {
    ROS_INFO("Getting actuator %s from ethercat actuators description ...", actuator_state->name_.c_str());

    // Lookup the actuator in the ethercat actuators description
    std::map<std::string, EthercatActuatorDescription>::const_iterator ethercat_actuator =
        ethercat_actuators_description.find(actuator_state->name_);

    if (ethercat_actuator == ethercat_actuators_description.end())
    {
      throw std::runtime_error(actuator_state->name_ + " could not be found in the ethercat actuators description");
    }

    actuators_.push_back(EthercatActuator(ethercat_actuator->second, ethercat_interface_, actuator_state));
  }

  // 3. Finally register all interfaces to ROS control
  transmission_manager_.registerInterfacesToROSControl(this);
}

void EthercatHardwareInterface::read(const ros::Time&, const ros::Duration& period)
{
  ethercat_interface_.receiveAll();

  for (EthercatActuator& actuator : actuators_)
  {
    actuator.read(period);
  }

  transmission_manager_.propogateAcuatorStatesToJointStates();
}

void EthercatHardwareInterface::write(const ros::Time&, const ros::Duration&)
{
  transmission_manager_.propogateJointStatesToActuatorStates();

  for (EthercatActuator& actuator : actuators_)
  {
    actuator.write();
  }

  ethercat_interface_.sendAll();
}
}
