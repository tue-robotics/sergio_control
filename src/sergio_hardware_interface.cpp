#include "sergio_hardware_interface.h"

#include <transmission_interface/transmission_parser.h>
#include <transmission_interface/transmission_loader.h>
#include <transmission_interface/simple_transmission_loader.h>

namespace sergio_control
{

SergioHardwareInterface::SergioHardwareInterface(const std::string& ethernet_interface, const std::string& urdf_string,
                                                 const std::map<std::string, EthercatActuatorDescription>& ethercat_actuators_description)
{
  // 1. Connect to the ethercat interface
  if (!ethercat_interface_.initialize(ethernet_interface))
  {
    throw std::runtime_error("Failed to initialize ethercat interface on ethernet interface " + ethernet_interface);
  }

  // 2. Parse the urdf and register all transmissions
  transmission_interface::TransmissionParser parser;
  std::vector<transmission_interface::TransmissionInfo> transmission_infos;
  if (!parser.parse(urdf_string, transmission_infos))
  {
    throw std::runtime_error("Failed to parse urdf file! " + urdf_string);
  }

  for (const transmission_interface::TransmissionInfo& transmission_info : transmission_infos)
  {
    ROS_INFO("Loading transmission '%s' of type '%s'", transmission_info.name_.c_str(), transmission_info.type_.c_str());

    boost::shared_ptr<transmission_interface::Transmission> transmission;
    if (transmission_info.type_ == "transmission_interface/SimpleTransmission")
    {
      transmission_interface::SimpleTransmissionLoader loader;
      transmission = loader.load(transmission_info);
    }
    // Optionally add more transmissions here
    else
    {
      throw std::runtime_error("Unsupported transmission type: " + transmission_info.type_);
    }

    if (!transmission)
    {
      throw std::runtime_error("Failed to load transmission of type: " + transmission_info.type_);
    }

    registerTransmission(transmission_info.name_, transmission, transmission_info.actuators_,
                         transmission_info.joints_, ethercat_actuators_description);
  }

  // 3. Finally register all interfaces to ROS control
  registerInterface(&actuator_state_interface_);
  registerInterface(&actuator_effort_interface_);
  registerInterface(&joint_state_interface_);
  registerInterface(&joint_effort_interface_);
  registerInterface(&actuator_to_joint_transmission_interface_);
  registerInterface(&joint_to_actuator_transmission_interface_);
}

void SergioHardwareInterface::registerTransmission(std::string transmission_name,
                                                   boost::shared_ptr<transmission_interface::Transmission> transmission,
                                                   std::vector<transmission_interface::ActuatorInfo> transmission_actuator_infos,
                                                   std::vector<transmission_interface::JointInfo> transmission_joint_infos,
                                                   std::map<std::string, EthercatActuatorDescription> ethercat_actuators_description)
{
  ROS_INFO("Registering transmission '%s' with %d actuators and %d joints", transmission_name.c_str(),
           (int) transmission->numActuators(), (int) transmission->numJoints());

  // Required for transmission interfaces
  transmission_interface::ActuatorData actuator_data;
  transmission_interface::JointData joint_data;
  transmission_interface::ActuatorData actuator_command_data;
  transmission_interface::JointData joint_command_data;

  // Register all actuators
  for (transmission_interface::ActuatorInfo actuator_info : transmission_actuator_infos)
  {
    // Create a new actuator
    actuators_.push_back(getActuator(actuator_info, ethercat_actuators_description));
    Actuator& actuator = actuators_.back();

    // Expose the actuator state
    hardware_interface::ActuatorStateHandle actuator_state_handle(actuator.data_.name_, &actuator.data_.position_,
                                                                  &actuator.data_.velocity_, &actuator.data_.effort_);
    actuator_state_interface_.registerHandle(actuator_state_handle);

    // Expose the actuator command interface
    actuator_effort_interface_.registerHandle(hardware_interface::ActuatorHandle(actuator_state_handle,
                                                                                 &actuator.data_.command_));

    // Required for transmission interface
    actuator_data.position.push_back(&actuator.data_.position_);
    actuator_data.velocity.push_back(&actuator.data_.velocity_);
    actuator_data.effort.push_back(&actuator.data_.effort_);
    actuator_command_data.effort.push_back(&actuator.data_.command_);

    ROS_INFO("Registered state and command interface for actuator '%s'", actuator_info.name_.c_str());
  }

  // Register all joints
  for (transmission_interface::JointInfo joint_info : transmission_joint_infos)
  {
    // Create a new actuator
    joints_.push_back(JointState(joint_info.name_));
    JointState& joint = joints_.back();

    // Expose the joint state
    hardware_interface::JointStateHandle joint_state_handle(joint.name_, &joint.position_,
                                                            &joint.velocity_, &joint.effort_);
    joint_state_interface_.registerHandle(joint_state_handle);

    // Expose the joint command interface
    joint_effort_interface_.registerHandle(hardware_interface::JointHandle(joint_state_handle, &joint.command_));

    // Required for transmission interface
    joint_data.position.push_back(&joint.position_);
    joint_data.velocity.push_back(&joint.velocity_);
    joint_data.effort.push_back(&joint.effort_);
    joint_command_data.effort.push_back(&joint.command_);

    ROS_INFO("Registered state and command interface for joint '%s'", joint_info.name_.c_str());
  }

  transmissions_.push_back(transmission);

  // Register transmissions
  actuator_to_joint_transmission_interface_.registerHandle(
        transmission_interface::ActuatorToJointPositionHandle(transmission_name, transmission.get(),
                                                              actuator_data, joint_data));
  ROS_INFO("Registered actuator to joint position transmission interface '%s'", transmission_name.c_str());

  joint_to_actuator_transmission_interface_.registerHandle(
        transmission_interface::JointToActuatorEffortHandle(transmission_name, transmission.get(),
                                                            actuator_command_data, joint_command_data));
  ROS_INFO("Registered joint to actuator effort transmission interface '%s'", transmission_name.c_str());
}

void SergioHardwareInterface::read(const ros::Time &, const ros::Duration &)
{
  ethercat_interface_.receiveAll();

  for (Actuator& actuator : actuators_)
  {
//    TODO: Update the actuator state
//    actuator.data_.position_ = actuator.encoder / ENCODER_COUNTS_PER_CYCLE;
  }

  actuator_to_joint_transmission_interface_.propagate();
}

void SergioHardwareInterface::write(const ros::Time &, const ros::Duration &)
{
  joint_to_actuator_transmission_interface_.propagate();

  for (const Actuator& actuator : actuators_)
  {
    ROS_INFO("Sending %.3f [Nm] as command to actuator %s", actuator.data_.command_, actuator.data_.name_.c_str());
//    TODO: Send reference to real actuator
//    actuator.motor.send(actuator.data_.effort_ * SOME_FACTOR_FOR_NM_TO_VOLTAGE);
  }

  ethercat_interface_.sendAll();
}

}
