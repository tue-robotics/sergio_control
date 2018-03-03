#include "sergio_hardware_interface.h"

#include <transmission_interface/transmission_parser.h>
#include <transmission_interface/transmission_loader.h>

SergioHardwareInterface::SergioHardwareInterface(const std::string& ethernet_interface, const std::string& urdf_string)
{
//  // Parse the urdf and look for transmissions
//  transmission_interface::TransmissionParser parser;
//  std::vector<transmission_interface::TransmissionInfo> transmission_infos;
//  if (!parser.parse(urdf_string, transmission_infos))
//  {
//    throw std::runtime_error("Failed to parse urdf file!");
//  }

//  for (const transmission_interface::TransmissionInfo& transmission_info : transmission_infos)
//  {
//    transmission_info.
//  }

  // Expose the actuator state
  hardware_interface::ActuatorStateHandle actuator_state_handle(ACTUATOR_NAME, &actuator_.position_,
                                                                &actuator_.velocity_, &actuator_.effort_);
  actuator_state_interface_.registerHandle(actuator_state_handle);

  // Expose the actuator command interface
  actuator_effort_interface_.registerHandle(hardware_interface::ActuatorHandle(actuator_state_handle,
                                                                                &actuator_.command_));

  // Expose the joint state
  hardware_interface::JointStateHandle joint_state_handle(JOINT_NAME, &joint_.position_,
                                                          &joint_.velocity_, &joint_.effort_);
  joint_state_interface_.registerHandle(joint_state_handle);

  // Expose the joint command interface
  joint_effort_interface_.registerHandle(hardware_interface::JointHandle(joint_state_handle, &joint_.command_));

  // Create transmission from actuator to joint

  // Required for transmission interfaces
  transmission_interface::ActuatorData actuator_data;
  transmission_interface::JointData joint_data;

  // Store actuator data for transmissions
  actuator_data.position.push_back(&actuator_.position_);
  actuator_data.velocity.push_back(&actuator_.velocity_);
  actuator_data.effort.push_back(&actuator_.effort_);

  // Store joint data for transmissions
  joint_data.position.push_back(&joint_.position_);
  joint_data.velocity.push_back(&joint_.velocity_);
  joint_data.effort.push_back(&joint_.effort_);

  // Create and store transmission
  actuator_to_joint_transmission_ = std::shared_ptr<transmission_interface::Transmission>(
        new transmission_interface::SimpleTransmission(66));
  actuator_to_joint_transmission_interface_.registerHandle(
        transmission_interface::ActuatorToJointPositionHandle(ACTUATOR_TO_JOINT_TRANSMISSION_NAME,
                                                              actuator_to_joint_transmission_.get(),
                                                              actuator_data, joint_data));

  // Create transmission from joint to actuator

  // Required for transmission interfaces
  transmission_interface::ActuatorData actuator_command_data;
  transmission_interface::JointData joint_command_data;

  // Store actuator data for transmissions
  actuator_command_data.effort.push_back(&actuator_.command_);

  // Store joint data for transmissions
  joint_command_data.effort.push_back(&joint_.command_);

  joint_to_actuator_transmission_ = std::shared_ptr<transmission_interface::Transmission>(
        new transmission_interface::SimpleTransmission(66));
  joint_to_actuator_transmission_interface_.registerHandle(
        transmission_interface::JointToActuatorEffortHandle(JOINT_TO_ACTUATOR_TRANSMISSION_NAME,
                                                            joint_to_actuator_transmission_.get(),
                                                            actuator_command_data, joint_command_data));

  // Finally register all interfaces
  registerInterface(&actuator_state_interface_);
  registerInterface(&actuator_effort_interface_);
  registerInterface(&joint_state_interface_);
  registerInterface(&joint_effort_interface_);
  registerInterface(&actuator_to_joint_transmission_interface_);
  registerInterface(&joint_to_actuator_transmission_interface_);

  // TODO: Configure the ethercat interface
}

void SergioHardwareInterface::read(const ros::Time &, const ros::Duration &)
{
  // TODO: Read the ethercat interface
  //
  //       1. Read the ethercat IO
  //       2. Convert the encoder counts to radians
  //       3. Write to the actuator state

  actuator_to_joint_transmission_interface_.propagate();
}

void SergioHardwareInterface::write(const ros::Time &, const ros::Duration &)
{
  joint_.command_ = 1.0;

  joint_to_actuator_transmission_interface_.propagate();

  std::cout << "Joint command: " << joint_.command_ << std::endl;
  std::cout << "Acuator command: " << actuator_.command_ << std::endl;
  // TODO: Write the ethercat interface
  //
  //       1. Convert the effort to voltage
  //       2. Write to ethercat IO
}
