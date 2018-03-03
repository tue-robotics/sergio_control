#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include "sergio_hardware_mapping.h"

class SergioHardwareInterface : public hardware_interface::RobotHW
{
public:

  //!
  //! \brief Sergio Robot hardware interface
  //!
  SergioHardwareInterface(const std::string& ethernet_interface);

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
  //! \brief joint_state_interface_ Exposes the joint interface to ROS Control
  //!
  hardware_interface::JointStateInterface joint_state_interface_;

  //!
  //! \brief effort_joint_interface Exposes an effort interface to ROS control
  //!
  hardware_interface::EffortJointInterface effort_joint_interface_;

  //!
  //! \brief joints_ Holds the joint state and the joint effort references
  //!
  struct Joint
  {
    double position_ = 0;
    double velocity_ = 0;
    double effort_ = 0;
    double effort_command_ = 0;

    Joint() = default;
  } joints_[NUM_JOINTS];

};
