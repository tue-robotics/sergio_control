#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <transmission_interface/simple_transmission.h>
#include <transmission_interface/differential_transmission.h>
#include <transmission_interface/transmission_interface.h>

#include "sergio_hardware_mapping.h"

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

  struct Actuator
  {
    // state
    double position_ = 0;
    double velocity_ = 0;
    double effort_ = 0;

    // reference
    double command_ = 0;

    Actuator() = default;
  } actuator_;

  struct Joint
  {
    // state
    double position_ = 0;
    double velocity_ = 0;
    double effort_ = 0;

    // reference
    double command_ = 0;

    Joint() = default;
  } joint_;

  std::shared_ptr<transmission_interface::Transmission> actuator_to_joint_transmission_;
  std::shared_ptr<transmission_interface::Transmission> joint_to_actuator_transmission_;
};
