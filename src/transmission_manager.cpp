//
// Copyright (c) 2018 TUe Robotics
//
// @author Rein Appeldoorn (reinzor)
//

#include "./transmission_manager.h"

#include <transmission_interface/transmission_parser.h>
#include <transmission_interface/transmission_loader.h>
#include <transmission_interface/simple_transmission_loader.h>
#include <transmission_interface/differential_transmission_loader.h>
#include <string>
#include <vector>

namespace transmission_manager
{
TransmissionManager::TransmissionManager(const std::string& urdf_string)
{
  // 2. Parse the urdf and register all transmissions
  transmission_interface::TransmissionParser parser;
  std::vector<transmission_interface::TransmissionInfo> transmission_infos;
  if (!parser.parse(urdf_string, transmission_infos))
  {
    throw std::runtime_error("Failed to parse urdf file! " + urdf_string);
  }

  for (const transmission_interface::TransmissionInfo& transmission_info : transmission_infos)
  {
    ROS_INFO("Loading transmission '%s' of type '%s'", transmission_info.name_.c_str(),
             transmission_info.type_.c_str());

    boost::shared_ptr<transmission_interface::Transmission> transmission;
    if (transmission_info.type_ == "transmission_interface/SimpleTransmission")
    {
      transmission_interface::SimpleTransmissionLoader loader;
      transmission = loader.load(transmission_info);
    }
    else if (transmission_info.type_ == "transmission_interface/DifferentialTransmission")
    {
      transmission_interface::DifferentialTransmissionLoader loader;
      transmission = loader.load(transmission_info);
    }
    // Optionally add more transmissions here
    else
    {
      throw std::runtime_error("Unsupported transmission type: " + transmission_info.type_);
    }

    if (!transmission)
    {
      throw std::runtime_error("Failed to load transmission of type: " + transmission_info.type_);
    }

    registerTransmission(transmission_info.name_, transmission, transmission_info.actuators_,
                         transmission_info.joints_);
  }

  ROS_INFO("TransmissionManager initialized %d transmissions", (int)transmission_infos.size());
}

void TransmissionManager::registerTransmission(
    std::string transmission_name, boost::shared_ptr<transmission_interface::Transmission> transmission,
    std::vector<transmission_interface::ActuatorInfo> transmission_actuator_infos,
    std::vector<transmission_interface::JointInfo> transmission_joint_infos)
{
  ROS_INFO("Registering transmission '%s' with %d actuators and %d joints", transmission_name.c_str(),
           (int)transmission->numActuators(), (int)transmission->numJoints());

  // Register all actuators
  transmission_interface::ActuatorData actuator_data;
  transmission_interface::ActuatorData actuator_command_data;

  for (transmission_interface::ActuatorInfo actuator_info : transmission_actuator_infos)
  {
    // Create a new actuator state and store this one
    ActuatorStatePtr actuator_state(new ActuatorState(actuator_info.name_));
    actuator_states_[actuator_info.name_] = actuator_state;

    // Expose the actuator state
    hardware_interface::ActuatorStateHandle actuator_state_handle(actuator_state->name_, &actuator_state->position_,
                                                                  &actuator_state->velocity_, &actuator_state->effort_);
    actuator_state_interface_.registerHandle(actuator_state_handle);

    // Expose the actuator command interface
    actuator_effort_interface_.registerHandle(
        hardware_interface::ActuatorHandle(actuator_state_handle, &actuator_state->command_));

    // Required for transmission interface
    actuator_data.position.push_back(&actuator_state->position_);
    actuator_data.velocity.push_back(&actuator_state->velocity_);
    actuator_data.effort.push_back(&actuator_state->effort_);
    actuator_command_data.effort.push_back(&actuator_state->command_);

    ROS_INFO("Registered state and command interface for actuator '%s'", actuator_info.name_.c_str());
  }

  // Register all joints
  transmission_interface::JointData joint_data;
  transmission_interface::JointData joint_command_data;

  for (transmission_interface::JointInfo joint_info : transmission_joint_infos)
  {
    // Create a new actuator
    JointStatePtr joint_state(new JointState(joint_info.name_));

    // Expose the joint state
    hardware_interface::JointStateHandle joint_state_handle(joint_state->name_, &joint_state->calibrated_position_,
                                                            &joint_state->velocity_, &joint_state->effort_);
    joint_state_interface_.registerHandle(joint_state_handle);

    // Expose the joint command interface
    joint_effort_interface_.registerHandle(hardware_interface::JointHandle(joint_state_handle, &joint_state->command_));

    // Required for transmission interface
    joint_data.position.push_back(&joint_state->raw_position_);  // Note that we pass the raw position here
    joint_data.velocity.push_back(&joint_state->velocity_);
    joint_data.effort.push_back(&joint_state->effort_);
    joint_command_data.effort.push_back(&joint_state->command_);

    ROS_INFO("Registered state and command interface for joint '%s'", joint_info.name_.c_str());
  }

  transmissions_.push_back(transmission);

  // Register transmissions
  actuator_to_joint_position_transmission_interface_.registerHandle(
      transmission_interface::ActuatorToJointPositionHandle(transmission_name, transmission.get(), actuator_data,
                                                            joint_data));
  actuator_to_joint_velocity_transmission_interface_.registerHandle(
      transmission_interface::ActuatorToJointVelocityHandle(transmission_name, transmission.get(), actuator_data,
                                                            joint_data));
  actuator_to_joint_effort_transmission_interface_.registerHandle(transmission_interface::ActuatorToJointEffortHandle(
      transmission_name, transmission.get(), actuator_data, joint_data));
  ROS_INFO("Registered actuator to joint position transmission interface '%s'", transmission_name.c_str());

  joint_to_actuator_effort_transmission_interface_.registerHandle(transmission_interface::JointToActuatorEffortHandle(
      transmission_name, transmission.get(), actuator_command_data, joint_command_data));
  ROS_INFO("Registered joint to actuator effort transmission interface '%s'", transmission_name.c_str());
}

void TransmissionManager::registerInterfacesToROSControl(hardware_interface::InterfaceManager* interface_manager)
{
  // Register the actuator interfaces for actuator state (output) and actuator effort (input)
  interface_manager->registerInterface(&actuator_state_interface_);
  interface_manager->registerInterface(&actuator_effort_interface_);

  // Register joint interfaces for the joint state (output) and joint effort (input)
  interface_manager->registerInterface(&joint_state_interface_);
  interface_manager->registerInterface(&joint_effort_interface_);

  // Register transmission interfaces for actuators to joints
  interface_manager->registerInterface(&actuator_to_joint_position_transmission_interface_);
  interface_manager->registerInterface(&actuator_to_joint_velocity_transmission_interface_);
  interface_manager->registerInterface(&actuator_to_joint_effort_transmission_interface_);

  // Register transmission interfaces for joints to actuators
  interface_manager->registerInterface(&joint_to_actuator_effort_transmission_interface_);
}

void TransmissionManager::calibrateJointPosition(const std::string& joint_name, double real_joint_position)
{
  // Verify whether the joint is present in the joint states map
  if (joint_states_.find(joint_name) == joint_states_.end())
  {
    throw std::runtime_error("Joint name '" + joint_name + "' not present in joint_states map");
  }
  else
  {
    joint_states_[joint_name]->position_offset_ = real_joint_position - joint_states_[joint_name]->raw_position_;
  }
}

void TransmissionManager::propagateAcuatorStatesToJointStates()
{
  // We read the state of the actuator and we need to propagate this to the corresponding joints
  actuator_to_joint_position_transmission_interface_.propagate();
  actuator_to_joint_velocity_transmission_interface_.propagate();
  actuator_to_joint_effort_transmission_interface_.propagate();

  // Now propogate the joint offsets
  for (auto joint_state : getJointStates())
  {
    joint_state->calibrated_position_ = joint_state->raw_position_ + joint_state->position_offset_;
  }
}

void TransmissionManager::propagateJointStatesToActuatorStates()
{
  // We control the joint position and the controller outputs an effort for the joint, this transmission makes sure
  // that we propagate this joint effort to the corresponding actuators
  joint_to_actuator_effort_transmission_interface_.propagate();
}

//!
//! \brief getMapItemsAsVector Helper function to return the items in a map as a vector
//! \param map The input map
//! \return Vector of all the items in the map
//!
template <typename K, typename V>
std::vector<V> getMapItemsAsVector(const std::map<K, V>& map)
{
  std::vector<V> output;
  for (auto iter : map)
  {
    output.push_back(iter.second);
  }
  return output;
}

std::vector<ActuatorStatePtr> TransmissionManager::getActuatorStates()
{
  return getMapItemsAsVector(actuator_states_);
}

std::vector<JointStatePtr> TransmissionManager::getJointStates()
{
  return getMapItemsAsVector(joint_states_);
}

}  // namespace transmission_manager
