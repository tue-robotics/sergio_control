//
// Copyright (c) 2018 TUe Robotics
//
// @author Rein Appeldoorn (reinzor)
//

#include "./ethercat_hardware_interface.h"

#include <ethercat_interface/exceptions.h>
#include <map>
#include <string>

namespace ethercat_hardware_interface
{
EthercatHardwareInterface::EthercatHardwareInterface(
    const std::string& interface_name, const std::string& urdf_string,
    const std::map<std::string, EthercatActuatorDescription>& ethercat_actuators_description,
    const std::string& package_name, const std::string& executable_name) :
  transmission_manager_(urdf_string)
{
  // 1. Connect to the ethercat interface
  try
  {
    interface_ = ethercat_interface::InterfacePtr(new ethercat_interface::Interface(interface_name));
  }
  catch (ethercat_interface::SocketException)
  {
    throw std::runtime_error("No socket connection on " + interface_name + ". Try excecuting the following "
                                                                               "command: sudo setcap cap_net_raw+ep "
                                                                               "$(readlink $(catkin_find " +
                             package_name + " " + executable_name + "))\n");
  }

  // 2. Get the actuators based on the description parsed from the URDF
  for (auto actuator_state : transmission_manager_.getActuatorStates())
  {
    ROS_INFO("Getting actuator %s from ethercat actuators description ...", actuator_state->name_.c_str());

    // Lookup the actuator in the ethercat actuators description
    std::map<std::string, EthercatActuatorDescription>::const_iterator ethercat_actuator =
        ethercat_actuators_description.find(actuator_state->name_);

    if (ethercat_actuator == ethercat_actuators_description.end())
    {
      throw std::runtime_error(actuator_state->name_ + " could not be found in the ethercat actuators description");
    }

    actuators_.push_back(EthercatActuator(ethercat_actuator->second, interface_, actuator_state));
  }

  // 3. Null all joints in the transmission manager
  read(ros::Time::now(), ros::Duration());
  for (auto joint_state : transmission_manager_.getJointStates())
  {
    transmission_manager_.calibrateJointPosition(joint_state->name_, 0);
  }

  // 4. Finally register all interfaces to ROS control
  transmission_manager_.registerInterfacesToROSControl(this);
}

void EthercatHardwareInterface::read(const ros::Time&, const ros::Duration& period)
{
  interface_->read();

  for (EthercatActuator& actuator : actuators_)
  {
    actuator.read(period);
  }

  transmission_manager_.propagateAcuatorStatesToJointStates();
}

void EthercatHardwareInterface::write(const ros::Time&, const ros::Duration&)
{
  transmission_manager_.propagateJointStatesToActuatorStates();

  for (EthercatActuator& actuator : actuators_)
  {
    actuator.write();
  }

  interface_->write();
}
}  // namespace ethercat_hardware_interface
