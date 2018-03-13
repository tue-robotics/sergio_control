#pragma once

#include <hardware_interface/robot_hw.h>
#include "./ethercat_interface_descriptions.h"
#include "./transmission_manager.h"

namespace sergio_control
{

struct Actuator
{
  Actuator(const EthercatActuatorDescription& description, ActuatorState* state, EthercatInterface& interface) :
    state_(state),
    description_(description)
  {
    ROS_INFO("Registering analogue out interface on slave %d and channel %d...",
             (int) description.motor_.slave_, (int) description.motor_.channel_);
    analogue_out_ = interface.getInterface(description.motor_.slave_, description.motor_.channel_);
    ROS_INFO("Registering digital in interface on slave %d and channel %d ...",
             (int) description.encoder_.slave_, (int) description.encoder_.channel_);
    encoder_in_ = interface.getInterface(description.encoder_.slave_, description.encoder_.channel_);
    ROS_INFO("Actuator initialized");
  }

  EthercatActuatorDescription description_;
  std::shared_ptr<IOInterface> analogue_out_;
  std::shared_ptr<IOInterface> encoder_in_;
  ActuatorState* state_;
};

class SergioHardwareInterface : public hardware_interface::RobotHW
{
public:

  //!
  //! \brief SergioHardwareInterface Hardware interface for the Sergio robot
  //! \param ethernet_interface Network address of the ethercat interface
  //! \param urdf_string URDF String used to parse transmission out of the robot model
  //! \param ethercat_actuators_description Available ethercat actuators; names should match the actuator names in urdf
  //!
  SergioHardwareInterface(const std::string& ethernet_interface, const std::string& urdf_string,
                          const std::map<std::string, EthercatActuatorDescription>& ethercat_actuators_description);

  //!
  //! \brief read Read data from ethercat interface
  //!
  void read(const ros::Time& /*time*/, const ros::Duration& /*period*/);

  //!
  //! \brief write Write data to ethercat interface
  //!
  void write(const ros::Time& /*time*/, const ros::Duration& /*period*/);

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
  std::vector<Actuator> actuators_;

};

}
