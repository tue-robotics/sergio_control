#include "sergio_hardware_interface.h"

SergioHardwareInterface::SergioHardwareInterface(const std::string& ethernet_interface)
{
  for (unsigned int i = 0; i < NUM_JOINTS; i++)
  {
    hardware_interface::JointStateHandle joint_state_handle(JOINT_NAMES[i],
                                                            &joints_[i].position_,
                                                            &joints_[i].velocity_,
                                                            &joints_[i].effort_);

    effort_joint_interface_.registerHandle(hardware_interface::JointHandle(joint_state_handle,
                                                                           &joints_[i].effort_command_));
  }
  registerInterface(&joint_state_interface_);
  registerInterface(&effort_joint_interface_);

  // TODO: Configure the ethercat interface
}

void SergioHardwareInterface::read(const ros::Time &, const ros::Duration &)
{
  // TODO: Read the ethercat interface
}

void SergioHardwareInterface::write(const ros::Time &, const ros::Duration &)
{
  // TODO: Write to the ethercat interface
}
