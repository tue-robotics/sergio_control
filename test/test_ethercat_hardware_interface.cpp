//
// Copyright (c) 2018 TUe Robotics
//
// @author Rein Appeldoorn (reinzor)
//

#include <string>
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>

#include "../src/ethercat_interface_descriptions.h"
#include "../src/ethercat_interface_parser.h"
#include "../src/ethercat_interfaces.h"
#include "../src/ethercat_hardware_interface.h"

using namespace ethercat_hardware_interface;

//!
//! \brief controlThread Separate thread for running the controller
//! \param rate Rate of the control loop
//! \param robot Pointer to the robot hardare interface
//! \param cm Controller manager interface to ROS control
//!
void controlThread(ros::Rate rate, EthercatHardwareInterface* robot,
                   controller_manager::ControllerManager* cm)
{
  ros::Time last_cycle_time = ros::Time::now();
  while (ros::ok())
  {
    robot->read(ros::Time::now(), ros::Time::now() - last_cycle_time);
    cm->update(ros::Time::now(), ros::Time::now() - last_cycle_time);
    robot->write(ros::Time::now(), ros::Time::now() - last_cycle_time);

    if (rate.cycleTime() > rate.expectedCycleTime())
    {
      ROS_WARN_STREAM_DELAYED_THROTTLE(10.0, "Cycle time too high: Cycle time: " << rate.cycleTime()
                                                                                 << ", Expected cycle time: "
                                                                                 << rate.expectedCycleTime());
    }

    last_cycle_time = ros::Time::now();

    rate.sleep();
  }
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "test_hardware_interface");

  ros::NodeHandle local_nh("~");
  ros::Rate rate(local_nh.param("rate", 50));
  std::string ethernet_interface = local_nh.param("ethercat_interface", std::string("eno1"));
  std::string urdf_string = local_nh.param("/robot_description", std::string(""));

  std::map<std::string, EthercatActuatorInterfaceDescription> ethercat_actuator_interfaces;
  try
  {
    XmlRpc::XmlRpcValue ethercat_actuator_interfaces_param;
    local_nh.getParam("ethercat_actuator_interfaces", ethercat_actuator_interfaces_param);
    ethercat_actuator_interfaces = getEthercatActuatorInterfacesDescription(ethercat_actuator_interfaces_param);
  }
  catch (const std::exception& e)
  {
    ROS_FATAL_STREAM("Failed to parse ~ethercat_actuator_interfaces parameter: " << e.what());
    return 1;
  }

  std::map<std::string, EthercatJointPositionInterfaceDescription> ethercat_joint_position_interfaces;
  try
  {
    XmlRpc::XmlRpcValue ethercat_joint_positions_param;
    local_nh.getParam("ethercat_absolute_joint_position_interfaces", ethercat_joint_positions_param);
    ethercat_joint_position_interfaces = getEthercatJointPositionInterfacesDescription(ethercat_joint_positions_param);
  }
  catch (const std::exception& e)
  {
    ROS_FATAL_STREAM("Failed to parse ~ethercat_absolute_joint_position_interfaces parameter: " << e.what());
    return 1;
  }

  std::vector<EthercatInterfaceDescription> ethercat_input_interfaces;
  try
  {
    XmlRpc::XmlRpcValue ethercat_input_interfaces_param;
    local_nh.getParam("ethercat_input_interfaces", ethercat_input_interfaces_param);
    ethercat_input_interfaces = getEthercatInterfaceDescription(ethercat_input_interfaces_param);
  }
  catch (const std::exception& e)
  {
    ROS_FATAL_STREAM("Failed to parse ~ethercat_input_interfaces parameter: " << e.what());
    return 1;
  }

  std::vector<EthercatInterfaceDescription> ethercat_output_interfaces;
  try
  {
    XmlRpc::XmlRpcValue ethercat_output_interfaces_param;
    local_nh.getParam("ethercat_output_interfaces", ethercat_output_interfaces_param);
    ethercat_output_interfaces = getEthercatInterfaceDescription(ethercat_output_interfaces_param);
  }
  catch (const std::exception& e)
  {
    ROS_FATAL_STREAM("Failed to parse ~ethercat_output_interfaces parameter: " << e.what());
    return 1;
  }

  try
  {
    EthercatHardwareInterface robot(ethernet_interface, urdf_string,

                                    ethercat_actuator_interfaces,
                                    ethercat_joint_position_interfaces,

                                    ethercat_input_interfaces,
                                    ethercat_output_interfaces,

                                    "sergio_control", "test_ethercat_hardware_interface");

    controller_manager::ControllerManager cm(&robot);

    boost::thread(boost::bind(controlThread, rate, &robot, &cm));

    ROS_INFO("Test Control initialized, spinning ...");

    ros::spin();
  }
  catch (const std::exception& e)
  {
    ROS_FATAL_STREAM("Test control error: " << e.what());
    return 1;
  }

  return 0;
}
