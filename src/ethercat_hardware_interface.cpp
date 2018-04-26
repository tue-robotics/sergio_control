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
EthercatHardwareInterface::EthercatHardwareInterface(const std::string& interface_name, const std::string& urdf_string,
    const std::map<std::string, EthercatActuatorInterfaceDescription>& actuator_interface_description,
    const std::map<std::string, EthercatJointPositionInterfaceDescription>& absolute_joint_position_interfaces_description,
    const std::string& package_name, const std::string& executable_name) :
  transmission_manager_(urdf_string, &ros_control_interfaces_)
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

  // 2. Load the ethercat actuators based on the provided description
  for (auto actuator_description : actuator_interface_description)
  {
    auto name = actuator_description.first;
    auto description = actuator_description.second;

    ROS_INFO("Constructing ethercat actuator for actuator %s ...", name.c_str());

    // Verify whether the actuator is present in the URDF
    if (transmission_manager_.actuator_states_.find(name) == transmission_manager_.actuator_states_.end())
    {
      throw std::runtime_error("Actuator " + name + " not present in robot URDF.");
    }

    auto actuator_state = transmission_manager_.actuator_states_[name];
    actuator_interfaces_.push_back(EthercatActuator(description, interface_, actuator_state));
  }

  // 3. Load the joint position interfaces based on the provided description
  for (auto absolute_joint_position_interface_description : absolute_joint_position_interfaces_description)
  {
    auto name = absolute_joint_position_interface_description.first;
    auto description = absolute_joint_position_interface_description.second;

    ROS_INFO("Constructing joint position interface for joint %s ...", name.c_str());

    // Verify whether the actuator is present in the URDF
    if (transmission_manager_.joint_states_.find(name) == transmission_manager_.joint_states_.end())
    {
      throw std::runtime_error("Joint " + name + " not present in robot URDF.");
    }

    JointStatePtr joint_state(new JointState(name + "_absolute"));
    absolute_joint_states_.push_back(joint_state);
    absolute_joint_position_interfaces_.push_back(EthercatJointPositionInterface(description, interface_, joint_state));

    ros_control_interfaces_.joint_state_interface_.registerHandle(
          hardware_interface::JointStateHandle(joint_state->name_, &joint_state->calibrated_position_,
                                               &joint_state->velocity_, &joint_state->effort_));
  }

  // Register the actuator interfaces for actuator state (output) and actuator effort (input)
  registerInterface(&ros_control_interfaces_.actuator_state_interface_);
  registerInterface(&ros_control_interfaces_.actuator_effort_interface_);

  // Register joint interfaces for the joint state (output) and joint effort (input)
  registerInterface(&ros_control_interfaces_.joint_state_interface_);
  registerInterface(&ros_control_interfaces_.joint_effort_interface_);

  // Register transmission interfaces for actuators to joints
  registerInterface(&ros_control_interfaces_.actuator_to_joint_position_transmission_interface_);
  registerInterface(&ros_control_interfaces_.actuator_to_joint_velocity_transmission_interface_);
  registerInterface(&ros_control_interfaces_.actuator_to_joint_effort_transmission_interface_);

  // Register transmission interfaces for joints to actuators
  registerInterface(&ros_control_interfaces_.joint_to_actuator_effort_transmission_interface_);

  // Register the joints limit interfaces
  registerInterface(&ros_control_interfaces_.effort_joint_saturation_joint_limits_interface_);
  registerInterface(&ros_control_interfaces_.effort_joint_soft_limits_joint_limits_interface_);

  //  Null all joints in the transmission manager
  read(ros::Time::now(), ros::Duration());
  for (auto joint_state : transmission_manager_.getJointStates())
  {
    initial_calibration_data_[joint_state->name_] = 0;
  }
  joint_calibration_data_buffer_.writeFromNonRT(initial_calibration_data_);

  // Advertise the service server
  ros::NodeHandle nh;
  calibrate_srv_ = nh.advertiseService("calibrate", &EthercatHardwareInterface::calibrateSrv, this);
}

bool EthercatHardwareInterface::calibrateSrv(control_msgs::CalibrateRequest& req, control_msgs::CalibrateResponse&)
{
  if (req.name.size() != req.position.size())
  {
    ROS_ERROR("Name and position vector are not of same size");
    return false;
  }

  std::map<std::string, double> joint_calibration_data;
  for (size_t i = 0; i < req.name.size(); ++i)
  {
    if (initial_calibration_data_.find(req.name[i]) == initial_calibration_data_.end())
    {
      ROS_ERROR("Name %s not present in transmission manager!", req.name[i].c_str());
      return false;
    }
    joint_calibration_data[req.name[i]] = req.position[i];
  }

  joint_calibration_data_buffer_.writeFromNonRT(joint_calibration_data);
  return true;
}

void EthercatHardwareInterface::read(const ros::Time&, const ros::Duration& period)
{
  interface_->read();

  // Process calibration data
  std::map<std::string, double>* calibration_data = joint_calibration_data_buffer_.readFromRT();
  if (calibration_data != last_joint_calibration_data)
  {
    for (auto iter : *calibration_data)
    {
      transmission_manager_.calibrateJointPosition(iter.first, iter.second);
    }
    last_joint_calibration_data = calibration_data;
  }

  for (EthercatActuator& actuator : actuator_interfaces_)
  {
    actuator.read(period);
  }

  for (EthercatJointPositionInterface& joint_position_interface : absolute_joint_position_interfaces_)
  {
    joint_position_interface.read();
  }

  transmission_manager_.propagateAcuatorStatesToJointStates();
}

void EthercatHardwareInterface::write(const ros::Time&, const ros::Duration& period)
{
  ros_control_interfaces_.effort_joint_saturation_joint_limits_interface_.enforceLimits(period);
  ros_control_interfaces_.effort_joint_soft_limits_joint_limits_interface_.enforceLimits(period);

  transmission_manager_.propagateJointStatesToActuatorStates();

  for (EthercatActuator& actuator : actuator_interfaces_)
  {
    actuator.write();
  }

  interface_->write();
}

}  // namespace ethercat_hardware_interface
