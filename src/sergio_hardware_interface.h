#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <transmission_interface/transmission_interface.h>
#include <transmission_interface/transmission_info.h>

#include "sergio_hardware_mapping.h"
#include "actuator_parser.h"

#include <ethercat_interface/ethercat_interface.h>

class SergioHardwareInterface : public hardware_interface::RobotHW
{
public:

  //!
  //! \brief Sergio Robot hardware interface
  //!
  SergioHardwareInterface(const std::string& ethernet_interface, const std::string& urdf_string);

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
  //! \param transmission_name Name of the transmission
  //! \param transmission Transmission ptr
  //! \param actuator_infos Information about the actuators in this transmission
  //! \param joint_infos Information about the joints in this transmission
  //!
  void registerTransmission(std::string transmission_name,
                            boost::shared_ptr<transmission_interface::Transmission> transmission,
                            std::vector<transmission_interface::ActuatorInfo> actuator_infos,
                            std::vector<transmission_interface::JointInfo> joint_infos);

  //!
  //! \brief actuators_ State and references of all registered actuators
  //!
  std::vector<Actuator> actuators_;

  //!
  //! \brief joints_ State and references of all registered joints
  //!
  std::vector<Data> joints_;

  //!
  //! \brief transmissions_ All transmissions obtained via the URDF
  //!
  std::vector<boost::shared_ptr<transmission_interface::Transmission> > transmissions_;

  //!
  //! \brief ethercat_interface_ IO interface
  //!
  EthercatInterface ethercat_interface_;
};
