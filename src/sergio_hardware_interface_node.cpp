//
// Copyright (c) 2018 TUe Robotics
//
// @author Rein Appeldoorn (reinzor)
//

#include <string>
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>

#include "./ethercat_actuator.h"
#include "./ethercat_actuator_parser.h"
#include "./ethercat_interface_descriptions.h"
#include "./sergio_hardware_interface.h"

using namespace sergio_control;

//!
//! \brief controlThread Separate thread for running the controller
//! \param rate Rate of the control loop
//! \param robot Pointer to the robot hardare interface
//! \param cm Controller manager interface to ROS control
//!
void controlThread(ros::Rate rate, SergioHardwareInterface* robot, controller_manager::ControllerManager* cm)
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
  ros::init(argc, argv, "sergio_hardware_interface_node");

  ros::NodeHandle local_nh("~");
  ros::Rate rate(local_nh.param("rate", 50));
  std::string ethernet_interface = local_nh.param("ethercat_interface", std::string("eno1"));
  std::string urdf_string = local_nh.param("/robot_description", std::string(""));

  std::map<std::string, ethercat_hardware_interface::EthercatMotorEncoderDescription> ethercat_actuators_description;
  try
  {
    XmlRpc::XmlRpcValue ethercat_actuators_param;
    local_nh.getParam("ethercat_actuators", ethercat_actuators_param);
    ethercat_actuators_description =
        ethercat_hardware_interface::getEthercatActuatorsDescription(ethercat_actuators_param);
  }
  catch (const std::exception& e)
  {
    ROS_FATAL_STREAM("Failed to parse ~ethercat_actuators parameter: " << e.what());
    return 1;
  }

  try
  {
    SergioHardwareInterface robot(ethernet_interface, urdf_string, ethercat_actuators_description);

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
