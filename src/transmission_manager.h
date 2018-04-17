#pragma once

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/internal/interface_manager.h>
#include <transmission_interface/transmission_interface.h>
#include <transmission_interface/transmission_info.h>

#include "./transmission_types.h"

namespace transmission_manager
{
class TransmissionManager
{
public:
  //!
  //! \brief TransmissionManager
  //! \param urdf_string
  //!
  TransmissionManager(const std::string& urdf_string);

  //!
  //! \brief registerInterfacesToROSControl
  //! \param interface_manager
  //!
  void registerInterfacesToROSControl(hardware_interface::InterfaceManager* interface_manager);

  //!
  //! \brief propogateAcuatorStatesToJointStates
  //!
  void propogateAcuatorStatesToJointStates();

  //!
  //! \brief propogateJointStatesToActuatorStates
  //!
  void propogateJointStatesToActuatorStates();

  //!
  //! \brief actuator_states_ State and commands of all ethercat actuators
  //!
  std::vector<std::shared_ptr<ActuatorState>> actuator_states_;

  //!
  //! \brief joint_states_ State and commands of all registered joints
  //!
  std::vector<std::shared_ptr<JointState>> joint_states_;

private:
  //!
  //! \brief registerTransmission Register a transmission for joint, actuator combinations
  //!
  //! This methods registers the transmissions parsed from the URDF. Per transmission the following:
  //!
  //! - Actuator state
  //! - Actuator command
  //! - Joint state
  //! - Joint command
  //! - Transmission from the reference from joint to actuator
  //! - Transmission from the state of the actuator to the joint state
  //!
  //! The transmissions make sure that the other half is updated when one half updates. So update the reference for the
  //! motor if the reference of the joint updates (command). But also update the state of the joint if the state of the
  //! actuator updates. This can be done by calling the  actuator_to_joint_transmission_interface_.propagate() and
  //! joint_to_actuator_transmission_interface_.propagate() methods.
  //!
  //! \param transmission_name Name of the transmission
  //! \param transmission Transmission ptr
  //! \param transmission_actuator_infos Information about the actuators in this transmission
  //! \param transmission_joint_infos Information about the joints in this transmission
  //!
  void registerTransmission(std::string transmission_name,
                            boost::shared_ptr<transmission_interface::Transmission> transmission,
                            std::vector<transmission_interface::ActuatorInfo> transmission_actuator_infos,
                            std::vector<transmission_interface::JointInfo> transmission_joint_infos);

  //!
  //! \brief actuator_state_interface_ Exposes the actuator interface to ROS Control
  //!
  hardware_interface::ActuatorStateInterface actuator_state_interface_;

  //!
  //! \brief actuator_command_interface_ Exposes an actuator interface to ROS control
  //!
  hardware_interface::EffortActuatorInterface actuator_effort_interface_;

  //!
  //! \brief joint_state_interface_ Exposes the joint interface to ROS Control
  //!
  hardware_interface::JointStateInterface joint_state_interface_;

  //!
  //! \brief joint_effort_interface_ Exposes the joint effort interface to ROS control
  //!
  hardware_interface::EffortJointInterface joint_effort_interface_;

  //!
  //! \brief actuator_to_joint_transmission_interface_ Actuator radians to joint radians
  //!
  transmission_interface::ActuatorToJointPositionInterface actuator_to_joint_position_transmission_interface_;

  //!
  //! \brief actuator_to_joint_velocity_transmission_interface_ Actuator radians / sec to joint radians / sec
  //!
  transmission_interface::ActuatorToJointVelocityInterface actuator_to_joint_velocity_transmission_interface_;

  //!
  //! \brief actuator_to_joint_effort_transmission_interface_ Actuator effort nm to joints effort nm
  //!
  transmission_interface::ActuatorToJointEffortInterface actuator_to_joint_effort_transmission_interface_;

  //!
  //! \brief joint_to_actuator_transmission_interface_ Joint efforts to actuator efforts
  //!
  transmission_interface::JointToActuatorEffortInterface joint_to_actuator_effort_transmission_interface_;

  //!
  //! \brief transmissions_ All transmissions obtained via the URDF
  //!
  std::vector<boost::shared_ptr<transmission_interface::Transmission>> transmissions_;
};
}
