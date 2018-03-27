#include <string>
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>

#include "../src/ethercat_actuator.h"
#include "../src/ethercat_actuator_parser.h"
#include "../src/ethercat_interface_descriptions.h"
#include "../src/transmission_manager.h"

#include <ethercat_interface/exceptions.h>

#include <hardware_interface/robot_hw.h>

//!
//! \brief The TestHardwareInterface class for testing the ethercat hardware interface with ROS control
//!
class TestHardwareInterface : public hardware_interface::RobotHW
{
public:
TestHardwareInterface(
    const std::string& ethernet_interface, const std::string& urdf_string,
    const std::map<std::string, EthercatActuatorDescription>& ethercat_actuators_description)
  : transmission_manager_(urdf_string)
{
  // 1. Connect to the ethercat interface
  try
  {
    if (!ethercat_interface_.initialize(ethernet_interface))
    {
      throw std::runtime_error("Failed to initialize ethercat interface on ethernet interface " + ethernet_interface);
    }
  }
  catch (SocketError)
  {
    throw std::runtime_error("No socket connection on " + ethernet_interface +
                             ". Try excecuting the following "
                             "command: sudo setcap cap_net_raw+ep $(readlink $(catkin_find "
                             "sergio_control test_hardware_interface))\n");
  }

  // 2. Get the actuators based on the description parsed from the URDF
  for (ActuatorState& actuator_state : transmission_manager_.actuator_states_)
  {
    ROS_INFO("Getting actuator %s from ethercat actuators description ...", actuator_state.name_.c_str());

    // Lookup the actuator in the ethercat actuators description
    std::map<std::string, EthercatActuatorDescription>::const_iterator ethercat_actuator =
        ethercat_actuators_description.find(actuator_state.name_);

    if (ethercat_actuator == ethercat_actuators_description.end())
    {
      throw std::runtime_error(actuator_state.name_ + " could not be found in the ethercat actuators description");
    }

    actuators_.push_back(EthercatActuator(ethercat_actuator->second, &actuator_state, ethercat_interface_));
  }

  // 3. Finally register all interfaces to ROS control
  transmission_manager_.registerInterfacesToROSControl(this);
}

//!
//! \brief read Read data from ethercat interface
//!
void read(const ros::Time&, const ros::Duration& period)
{
  ethercat_interface_.receiveAll();

  for (EthercatActuator& actuator : actuators_)
  {
    actuator.read(period);
  }

  transmission_manager_.propogateAcuatorStatesToJointStates();
}

//!
//! \brief write Write data to ethercat interface
//!
void write(const ros::Time&, const ros::Duration&)
{
  transmission_manager_.propogateJointStatesToActuatorStates();

  for (EthercatActuator& actuator : actuators_)
  {
    actuator.write();
  }

  ethercat_interface_.sendAll();
}

private:
  //!
  //! \brief transmission_manager_ Manages the transmissions and states of all actuators and joints
  //!
  TransmissionManager transmission_manager_;

  //!
  //! \brief ethercat_interface_ IO interface
  //!
  EthercatInterface ethercat_interface_;

  //!
  //! \brief actuators_ Holds the ethercat actuators and a pointer to the state
  //!
  std::vector<EthercatActuator> actuators_;
};

//!
//! \brief controlThread Separate thread for running the controller
//! \param rate Rate of the control loop
//! \param robot Pointer to the robot hardare interface
//! \param cm Controller manager interface to ROS control
//!
void controlThread(ros::Rate rate, TestHardwareInterface* robot, controller_manager::ControllerManager* cm)
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

  std::map<std::string, EthercatActuatorDescription> ethercat_actuators_description;
  try
  {
    XmlRpc::XmlRpcValue ethercat_actuators_param;
    local_nh.getParam("ethercat_actuators", ethercat_actuators_param);
    ethercat_actuators_description = getEthercatActuatorsDescription(ethercat_actuators_param);
  }
  catch (const std::exception& e)
  {
    ROS_FATAL_STREAM("Failed to parse ~ethercat_actuators parameter: " << e.what());
    return 1;
  }

  try
  {
    TestHardwareInterface robot(ethernet_interface, urdf_string, ethercat_actuators_description);

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
