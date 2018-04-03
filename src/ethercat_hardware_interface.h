#pragma once

#include <hardware_interface/robot_hw.h>
#include "./ethercat_interface_descriptions.h"
#include "./ethercat_actuator.h"
#include "./transmission_manager.h"

namespace ethercat_hardware_interface
{
class EthercatHardwareInterface : public hardware_interface::RobotHW
{
public:
  //!
  //! \brief EthercatHardwareInterface Hardware interface for a robot that can be controller with ethercat
  //! \param ethernet_interface Network address of the ethercat interface
  //! \param urdf_string URDF String used to parse transmission out of the robot model
  //! \param ethercat_actuators_description Available ethercat actuators; names should match the actuator names in urdf
  //!
  EthercatHardwareInterface(const std::string& ethernet_interface, const std::string& urdf_string,
                            const std::map<std::string, EthercatActuatorDescription>& ethercat_actuators_description);

  //!
  //! \brief read Read data from ethercat interface
  //!
  void read(const ros::Time& /*time*/, const ros::Duration& period);

  //!
  //! \brief write Write data to ethercat interface
  //!
  void write(const ros::Time& /*time*/, const ros::Duration& /*period*/);

private:
  //!
  //! \brief transmission_manager_ Manages the transmissions and states of all actuators and joints
  //!
  transmission_manager::TransmissionManager transmission_manager_;

  //!
  //! \brief ethercat_interface_ IO interface
  //!
  EthercatInterface ethercat_interface_;

  //!
  //! \brief actuators_ Holds the ethercat actuators and a pointer to the state
  //!
  std::vector<EthercatActuator> actuators_;
};
}
