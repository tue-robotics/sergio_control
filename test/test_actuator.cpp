//
// Copyright (c) 2018 TUe Robotics
//
// @author Rein Appeldoorn (reinzor)
//

#include <ros/ros.h>
#include <ethercat_interface/interface.h>
#include <ethercat_interface/exceptions.h>
#include "../src/ethercat_actuator.h"

//!
//! \brief getParam Get parameter from ROS nodehandle
//! \param name Parameter name
//! \param default_value Default value if not found
//! \param param_nh Node handle reference
//! \return The parameter
//!
template <typename T>
inline T getParam(const std::string& name, const T& default_value, ros::NodeHandle* param_nh)
{
  if (!param_nh->hasParam(name))
  {
    ROS_WARN_STREAM("Parameter '" << name << "' in namespace '" << param_nh->getNamespace()
                                  << "' not specified, defaults to '" << default_value << "'");
  }
  T param = param_nh->param(name, default_value);
  ROS_INFO_STREAM("Parameter '" << name << "' = '" << param << "'");
  return param;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_actuator");

  ros::NodeHandle local_nh("~");

  try
  {
    ethercat_interface::InterfacePtr interface(
          new ethercat_interface::Interface(getParam("ethercat_interface", std::string("eno1"), &local_nh)));

    ethercat_hardware_interface::EthercatActuatorDescription description;
    description.motor_.slave_ = getParam("motor_slave", 1, &local_nh);
    description.motor_.channel_ = getParam("motor_channel", 0, &local_nh);
    description.motor_.volt_per_newton_meter_ = getParam("motor_volt_per_newton_meter", 22.801652754998294, &local_nh);
    description.encoder_.slave_ = getParam("encoder_slave", 2, &local_nh);
    description.encoder_.channel_ = getParam("encoder_channel", 0, &local_nh);
    description.encoder_.encoder_counts_per_revolution_ = getParam("encoder_counts_per_revolution", 1024, &local_nh);

    std::shared_ptr<ActuatorState> actuator_state(new ActuatorState("actuator"));
    ethercat_hardware_interface::EthercatActuator ethercat_actuator(description, interface, actuator_state);

    double mechanical_reduction = getParam("mechanical_reduction", 66.2204081633, &local_nh);
    actuator_state->command_ = getParam("command", 0.05, &local_nh) / mechanical_reduction;

    ros::Rate rate(getParam("rate", 500, &local_nh));
    ros::Time last_cycle_time = ros::Time::now();
    while (ros::ok())
    {
      interface->read();

      ethercat_actuator.read(ros::Time::now() - last_cycle_time);
      ROS_INFO_DELAYED_THROTTLE(0.1, "Actuator position: %.8f", actuator_state->position_);
      ROS_INFO_DELAYED_THROTTLE(0.1, "Joint position: %.8f", actuator_state->position_ / mechanical_reduction);

      ROS_INFO_DELAYED_THROTTLE(0.1, "Writing %.8f Nm to joint", actuator_state->command_ * mechanical_reduction);
      ROS_INFO_DELAYED_THROTTLE(0.1, "Writing %.8f Nm to actuator", actuator_state->command_);
      ethercat_actuator.write();

      interface->write();

      last_cycle_time = ros::Time::now();
      rate.sleep();
    }
  }
  catch (ethercat_interface::SocketException)
  {
    ROS_FATAL_STREAM("SocketError. Try excecuting the following command: sudo setcap cap_net_raw+ep "
                     "$(readlink $(catkin_find sergio_control test_actuator))");
    return 1;
  }
  catch (const std::exception& e)
  {
    ROS_FATAL_STREAM("Failed to initialize ethercat interface: " << e.what());
    return 1;
  }

  return 0;
}
