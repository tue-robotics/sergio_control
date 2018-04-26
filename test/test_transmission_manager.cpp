//
// Copyright (c) 2018 TUe Robotics
//
// @author Rein Appeldoorn (reinzor)
//

#include <ros/ros.h>

#include "../src/ethercat_actuator_parser.h"
#include "../src/transmission_manager.h"

void setActuatorStatesPosition(std::vector<std::shared_ptr<ActuatorState>> states, double position, double velocity,
                               double effort)
{
  ROS_INFO("Setting actuator states position=%.2f, velocity=%.3f, effort=%.2f", position, velocity, effort);
  for (auto state : states)
  {
    state->position_ = position;
    state->velocity_ = velocity;
    state->effort_ = effort;
  }
}

void setJointStatesCommand(std::vector<std::shared_ptr<JointState>> states, double command)
{
  ROS_INFO("Setting joint states command to %.2f", command);
  for (auto state : states)
  {
    state->command_ = command;
  }
}

void printState(std::vector<std::shared_ptr<JointState>> joint_states,
                std::vector<std::shared_ptr<ActuatorState>> actuator_states)
{
  ROS_INFO("-------");
  for (auto joint_state : joint_states)
  {
    ROS_INFO_STREAM(*joint_state);
  }
  for (auto actuator_state : actuator_states)
  {
    ROS_INFO_STREAM(*actuator_state);
  }
  ROS_INFO("-------");
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "test_transmission_manager");

  ros::NodeHandle local_nh("~");
  std::string urdf_string = local_nh.param("/robot_description", std::string(""));

  transmission_manager::TransmissionManager transmission_manager(urdf_string);

  // First check if we can propagate the command on the joint to the command of the actuator
  setJointStatesCommand(transmission_manager.getJointStates(), 0.1);
  transmission_manager.propagateJointStatesToActuatorStates();
  printState(transmission_manager.getJointStates(), transmission_manager.getActuatorStates());

  setJointStatesCommand(transmission_manager.getJointStates(), -0.1);
  transmission_manager.propagateJointStatesToActuatorStates();
  printState(transmission_manager.getJointStates(), transmission_manager.getActuatorStates());

  setActuatorStatesPosition(transmission_manager.getActuatorStates(), 3, 2, 1);
  transmission_manager.propagateAcuatorStatesToJointStates();
  printState(transmission_manager.getJointStates(), transmission_manager.getActuatorStates());
}
