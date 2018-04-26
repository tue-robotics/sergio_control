#pragma once

#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/actuator_state_interface.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>

#include <transmission_interface/transmission_interface.h>

#include <joint_limits_interface/joint_limits_interface.h>


struct ROSControlInterfaces
{
  //!
  //! \brief actuator_state_interface_ Exposes the actuator interface to ROS Control
  //!
  hardware_interface::ActuatorStateInterface actuator_state_interface_;

  //!
  //! \brief actuator_command_interface_ Exposes an actuator interface to ROS control
  //!
  hardware_interface::EffortActuatorInterface actuator_effort_interface_;

  //!
  //! \brief joint_effort_interface_ Exposes the joint effort interface to ROS control
  //!
  hardware_interface::EffortJointInterface joint_effort_interface_;

  //!
  //! \brief joint_state_interface_ Exposes the joint interface to ROS Control
  //!
  hardware_interface::JointStateInterface joint_state_interface_;

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
  //! \brief effort_joint_saturation_interface_ Makes sure that the effort limits are not exceeded
  //!
  joint_limits_interface::EffortJointSaturationInterface effort_joint_saturation_joint_limits_interface_;

  //!
  //! \brief effort_joint_soft_limits_interface_ Ensure soft limits of joint
  //!
  joint_limits_interface::EffortJointSoftLimitsInterface effort_joint_soft_limits_joint_limits_interface_;
};
