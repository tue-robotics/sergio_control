#include <ros/ros.h>

#include "../src/ethercat_actuator_parser.h"
#include "../src/transmission_manager.h"

void setJointStatesPosition(std::vector<JointState>& states, double position)
{
  ROS_INFO("Setting joint states position to %.2f", position);
  for (JointState& state : states)
  {
    state.position_ = position;
  }
}

void printState(std::vector<JointState> joint_states, std::vector<ActuatorState> actuator_states)
{
  ROS_INFO("-------");
  for (JointState& joint_state : joint_states)
  {
    ROS_INFO_STREAM(joint_state);
  }
  for (ActuatorState& actuator_state : actuator_states)
  {
    ROS_INFO_STREAM(actuator_state);
  }
  ROS_INFO("-------");
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "test_transmission_manager");

  ros::NodeHandle local_nh("~");
  std::string urdf_string = local_nh.param("/robot_description", std::string(""));

  TransmissionManager transmission_manager(urdf_string);

  setJointStatesPosition(transmission_manager.joint_states_, 0.1);
  transmission_manager.propogateJointStatesToActuatorStates();
  printState(transmission_manager.joint_states_, transmission_manager.actuator_states_);
}
