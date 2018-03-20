#include <ros/ros.h>
#include <ethercat_interface/ethercat_interface.h>
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

  EthercatInterface ethercat_interface;
  try
  {
    ethercat_interface.initialize(getParam("ethercat_interface", std::string("eno1"), &local_nh));
  }
  catch (SocketError)
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

  EthercatActuatorDescription description;
  description.motor_.slave_ = getParam("motor_slave", 1, &local_nh);
  description.motor_.channel_ = getParam("motor_channel", 0, &local_nh);
  description.motor_.volt_per_newton_meter_ = getParam("motor_volt_per_newton_meter", 43.808411214953274, &local_nh);
  description.encoder_.slave_ = getParam("encoder_slave", 2, &local_nh);
  description.encoder_.channel_ = getParam("encoder_channel", 0, &local_nh);
  description.encoder_.encoder_counts_per_revolution_ = getParam("encoder_counts_per_revolution", 256, &local_nh);

  ActuatorState actuator_state("actuator");
  EthercatActuator ethercat_actuator(description, &actuator_state, ethercat_interface);

  actuator_state.command_ = getParam("command", 0.01, &local_nh);
  double throttle_period = getParam("throttle_period", 1.0, &local_nh);

  ros::Rate rate(getParam("rate", 1000, &local_nh));
  while (ros::ok())
  {
    ethercat_interface.receiveAll();

    ethercat_actuator.read();
    ROS_INFO_THROTTLE(throttle_period, "Actuator position: %.4f", actuator_state.position_);

    ROS_INFO_THROTTLE(throttle_period, "Writing %.4f Nm ...", actuator_state.command_);
    ethercat_actuator.write();

    ethercat_interface.sendAll();
    rate.sleep();
  }

  return 0;
}
