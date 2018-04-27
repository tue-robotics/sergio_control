//
// Copyright (c) 2018 TUe Robotics
//
// @author Rein Appeldoorn (reinzor)
//

#pragma once

#include "./ros_control_interfaces.h"

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/internal/interface_manager.h>
#include <transmission_interface/transmission_interface.h>
#include <transmission_interface/transmission_info.h>
#include <urdf/model.h>
#include <string>
#include <vector>

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
  explicit TransmissionManager(const std::string& urdf_string,
                               ROSControlInterfaces* ros_control_interfaces);

  //!
  //! \brief calibrateJointPosition Calibrates the joint position to the passed value
  //!
  //! Makes sure that the joint state position is set to the passed value. Required for external calibration
  //!
  //! \param joint_name The joint name
  //! \param real_joint_position The real joint position
  //!
  void calibrateJointPosition(const std::string& joint_name, double real_joint_position);

  //!
  //! \brief propagateAcuatorStatesToJointStates
  //!
  void propagateAcuatorStatesToJointStates();

  //!
  //! \brief propagateJointStatesToActuatorStates
  //!
  void propagateJointStatesToActuatorStates();

  //!
  //! \brief getActuatorStates Returns the actuator states vector
  //!
  std::vector<ActuatorStatePtr> getActuatorStates();

  //!
  //! \brief getJointStates Returns the joint states vector
  //!
  std::vector<JointStatePtr> getJointStates();

  //!
  //! \brief actuator_states_ State and commands of all ethercat actuators
  //!
  std::map<std::string, ActuatorStatePtr> actuator_states_;

  //!
  //! \brief joint_states_ State and commands of all registered joints
  //!
  std::map<std::string, JointStatePtr> joint_states_;

  //!
  //! \brief getJointActuatorNames returns all actuator names corresponding to the requested joint name
  //! \param joint_name joint name
  //! \return
  //!
  std::set<std::string> &getJointActuatorNames(const std::string& joint_name);

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
  //! - Per joint, joint limits
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
  //! \param urdf_model The urdf of the robot model
  //!
  void registerTransmission(std::string transmission_name,
                            boost::shared_ptr<transmission_interface::Transmission> transmission,
                            std::vector<transmission_interface::ActuatorInfo> transmission_actuator_infos,
                            std::vector<transmission_interface::JointInfo> transmission_joint_infos,
                            const urdf::Model& urdf_model);

  //!
  //! \brief ros_control_interfaces_ Interfaces to ROS control
  //!
  ROSControlInterfaces* ros_control_interfaces_;

  //!
  //! \brief transmissions_ All transmissions obtained via the URDF
  //!
  std::vector<boost::shared_ptr<transmission_interface::Transmission>> transmissions_;

  //!
  //! \brief joint_actuator_map_ Maps joint names to corresponding actuator names
  //!
  std::map<std::string, std::set<std::string> > joint_actuator_map_;
};
}  // namespace transmission_manager
