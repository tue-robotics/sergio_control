//
// Copyright (c) 2018 TUe Robotics
//
// @author Rein Appeldoorn (reinzor)
//

#pragma once

#include <hardware_interface/robot_hw.h>
#include <map>
#include <string>
#include <vector>

#include "./ethercat_interface_descriptions.h"
#include "./ethercat_interfaces.h"
#include "./transmission_manager.h"

#include "./ros_control_interfaces.h"

namespace ethercat_hardware_interface
{
class EthercatHardwareInterface : public hardware_interface::RobotHW
{
public:
  //!
  //! \brief EthercatHardwareInterface Hardware interface for a robot that can be controller with ethercat
  //! \param interface_name Name of the ethercat network interface
  //! \param urdf_string URDF String used to parse transmission out of the robot model
  //! \param ethercat_actuator_interfaces_description Available ethercat actuators; names should match the actuator names in urdf
  //! \param ethercat_joint_position_interfaces_description Available ethercat joint position interfaces; names should match the joint names in the urdf
  //! \param package_name ROS package name
  //! \param executable_name Name of the executable within the ROS package
  //!
  EthercatHardwareInterface(const std::string& interface_name, const std::string& urdf_string,
                            const std::map<std::string, EthercatActuatorInterfaceDescription>& actuator_interface_description,
                            const std::map<std::string, EthercatJointPositionInterfaceDescription>& absolute_joint_position_interfaces_description,
                            const std::string& package_name, const std::string& executable_name);

  //!
  //! \brief read Read data from ethercat interface
  //!
  void read(const ros::Time& /*time*/, const ros::Duration& period);

  //!
  //! \brief write Write data to ethercat interface
  //!
  void write(const ros::Time& /*time*/, const ros::Duration& /*period*/);

  //!
  //! \brief init
  //! \param root_nh
  //! \param robot_hw_nh
  //! \return
  //!
  bool init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh);

private:
  //!
  //! \brief interface_ IO interface
  //!
  ethercat_interface::InterfacePtr interface_;

private:
  ROSControlInterfaces ros_control_interfaces_;

private:
  //!
  //! \brief transmission_manager_ Manages the transmissions and states of all actuators and joints
  //!
  transmission_manager::TransmissionManager transmission_manager_;

  //!
  //! \brief actuators_ Holds the ethercat actuators and a pointer to the state
  //!
  std::vector<EthercatActuator> actuator_interfaces_;

private:
  //!
  //! \brief joint_states_
  //!
  std::vector<JointStatePtr> absolute_joint_states_;

  //!
  //! \brief actuators_ Holds the ethercat actuators and a pointer to the state
  //!
  std::vector<EthercatJointPositionInterface> absolute_joint_position_interfaces_;

};
}  // namespace ethercat_hardware_interface
