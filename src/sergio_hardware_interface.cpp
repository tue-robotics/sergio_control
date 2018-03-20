#include "sergio_hardware_interface.h"

#include <ethercat_interface/exceptions.h>

#include <transmission_interface/transmission_parser.h>
#include <transmission_interface/transmission_loader.h>
#include <transmission_interface/simple_transmission_loader.h>

namespace sergio_control
{
SergioHardwareInterface::SergioHardwareInterface(
    const std::string& ethernet_interface, const std::string& urdf_string,
    const std::map<std::string, EthercatActuatorDescription>& ethercat_actuators_description)
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
    throw std::runtime_error("No socket connection on " + ethernet_interface +
                             ". Try excecuting the following "
                             "command: sudo setcap cap_net_raw+ep $(readlink $(catkin_find "
                             "sergio_control sergio_control_node))\n");
  }

  // 2. Get the actuators based on the description parsed from the URDF
  for (ActuatorState& actuator_state : transmission_manager_.actuator_states_)
  {
    ROS_INFO("Getting actuator %s from ethercat actuators description ...", actuator_state.name_.c_str());

    // Lookup the actuator in the ethercat actuators description
    std::map<std::string, EthercatActuatorDescription>::const_iterator ethercat_actuator =
        ethercat_actuators_description.find(actuator_state.name_);

    if (ethercat_actuator == ethercat_actuators_description.end())
    {
      throw std::runtime_error(actuator_state.name_ + " could not be found in the ethercat actuators description");
    }

    actuators_.push_back(Actuator(ethercat_actuator->second, &actuator_state, ethercat_interface_));
  }

  // 3. Finally register all interfaces to ROS control
  transmission_manager_.registerInterfacesToROSControl(this);
}

void SergioHardwareInterface::read(const ros::Time&, const ros::Duration&)
{
  ethercat_interface_.receiveAll();

  for (Actuator& actuator : actuators_)
  {
    actuator.state_->position_ =
        (double)actuator.encoder_in_->read() / actuator.description_.encoder_.encoder_counts_per_revolution_;
    ROS_INFO("Read actuator position: %.2f from actuator %s", actuator.state_->position_,
             actuator.state_->name_.c_str());
  }

  transmission_manager_.propogateAcuatorStatesToJointStates();
}

void SergioHardwareInterface::write(const ros::Time&, const ros::Duration&)
{
  transmission_manager_.propogateJointStatesToActuatorStates();

  for (const Actuator& actuator : actuators_)
  {
    double voltage = actuator.state_->command_ * actuator.description_.motor_.volt_per_newton_meter_;
    ROS_INFO("Sending %.5f [Nm] %.5f [Volt] as command to actuator %s", actuator.state_->command_, voltage,
             actuator.state_->name_.c_str());
    actuator.analogue_out_->write(voltage);
  }

  ethercat_interface_.sendAll();
}
}
