#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <transmission_interface/transmission_interface.h>
#include <transmission_interface/transmission_info.h>

#include <ethercat_interface/ethercat_interface.h>

#include "sergio_hardware_mapping.h"
#include "actuator_parser.h"

namespace sergio_control
{

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
  //! \brief actuator_state_interface_ Exposes the actuator interface to ROS Control
  //!
  hardware_interface::ActuatorStateInterface actuator_state_interface_;

  //!
  //! \brief actuator_command_interface_ Exposes an actuator interface to ROS control
  //!
  hardware_interface::EffortActuatorInterface actuator_effort_interface_;

  //!
  //! \brief joint_state_interface_ Exposes the joint interface to ROS Control
  //!
  hardware_interface::JointStateInterface joint_state_interface_;

  //!
  //! \brief joint_effort_interface_ Exposes the joint effort interface to ROS control
  //!
  hardware_interface::EffortJointInterface joint_effort_interface_;

  //!
  //! \brief actuator_to_joint_transmission_interface_ Actuator radians to joint radians
  //!
  transmission_interface::ActuatorToJointPositionInterface actuator_to_joint_transmission_interface_;

  //!
  //! \brief joint_to_actuator_transmission_interface_ Joint efforts to actuator efforts
  //!
  transmission_interface::JointToActuatorEffortInterface joint_to_actuator_transmission_interface_;

  //!
  //! \brief registerTransmission Register a transmission for joint, actuator combinations
  //!
  //! This methods registers the transmissions parsed from the URDF. Per transmission the following:
  //!
  //! - Actuator state
  //! - Actuator command
  //! - Joint state
  //! - Joint command
  //! - Transmission from the reference from joint to actuator
  //! - Transmission from the state of the actuator to the joint state
  //!
  //! The transmissions make sure that the other half is updated when one half updates. So update the reference for the
  //! motor if the reference of the joint updates (command). But also update the state of the joint if the state of the
  //! actuator updates. This is done in the read / write hook by the transmission propogate methods.
  //!
  //! \param transmission_name Name of the transmission
  //! \param transmission Transmission ptr
  //! \param transmission_actuator_infos Information about the actuators in this transmission
  //! \param transmission_joint_infos Information about the joints in this transmission
  //! \param ethercat_actuators_description Description of the ethercat actuators
  //!
  void registerTransmission(std::string transmission_name,
                            boost::shared_ptr<transmission_interface::Transmission> transmission,
                            std::vector<transmission_interface::ActuatorInfo> transmission_actuator_infos,
                            std::vector<transmission_interface::JointInfo> transmission_joint_infos,
                            std::map<std::string, EthercatActuatorDescription> ethercat_actuators_description);

  //!
  //! \brief actuators_ State and references of all ethercat actuators
  //!
  std::vector<Actuator> actuators_;

  //!
  //! \brief joints_ State and references of all registered joints
  //!
  std::vector<JointState> joints_;

  //!
  //! \brief transmissions_ All transmissions obtained via the URDF
  //!
  std::vector<boost::shared_ptr<transmission_interface::Transmission>> transmissions_;

  //!
  //! \brief ethercat_interface_ IO interface
  //!
  EthercatInterface ethercat_interface_;
};

}
