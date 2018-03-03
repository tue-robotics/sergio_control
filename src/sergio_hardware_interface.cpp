#include "sergio_hardware_interface.h"

#include <transmission_interface/transmission_parser.h>
#include <transmission_interface/simple_transmission_loader.h>

SergioHardwareInterface::SergioHardwareInterface(const std::string& ethernet_interface, const std::string& urdf_string)
{
  // Parse the urdf and look for transmissions
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

    registerTransmission(transmission_info.name_, transmission, transmission_info.actuators_, transmission_info.joints_);
  }

  // Finally register all interfaces
  ROS_INFO("Registering all interfaces with ROS_CONTROL");
  registerInterface(&actuator_state_interface_);
  registerInterface(&actuator_effort_interface_);
  registerInterface(&joint_state_interface_);
  registerInterface(&joint_effort_interface_);
  registerInterface(&actuator_to_joint_transmission_interface_);
  registerInterface(&joint_to_actuator_transmission_interface_);

  // TODO: Configure the ethercat interface
}

void SergioHardwareInterface::registerTransmission(std::string transmission_name,
                                                   boost::shared_ptr<transmission_interface::Transmission> transmission,
                                                   std::vector<transmission_interface::ActuatorInfo> actuator_infos,
                                                   std::vector<transmission_interface::JointInfo> joint_infos)
{
  ROS_INFO("Registering transmission '%s' with %d actuators and %d joints", transmission_name.c_str(),
           (int) transmission->numActuators(), (int) transmission->numJoints());

  // Required for transmission interfaces
  transmission_interface::ActuatorData actuator_data;
  transmission_interface::JointData joint_data;
  transmission_interface::ActuatorData actuator_command_data;
  transmission_interface::JointData joint_command_data;

  // Register all actuators
  for (transmission_interface::ActuatorInfo actuator_info : actuator_infos)
  {
    // Create a new actuator
    actuators_.push_back(Data(actuator_info.name_));
    Data& actuator = actuators_.back();

    // Expose the actuator state
    hardware_interface::ActuatorStateHandle actuator_state_handle(actuator.name_, &actuator.position_,
                                                                  &actuator.velocity_, &actuator.effort_);
    actuator_state_interface_.registerHandle(actuator_state_handle);

    // Expose the actuator command interface
    actuator_effort_interface_.registerHandle(hardware_interface::ActuatorHandle(actuator_state_handle,
                                                                                 &actuator.command_));

    // Required for transmission interface
    actuator_data.position.push_back(&actuator.position_);
    actuator_data.velocity.push_back(&actuator.velocity_);
    actuator_data.effort.push_back(&actuator.effort_);
    actuator_command_data.effort.push_back(&actuator.command_);

    ROS_INFO("Registered state and command interface for actuator '%s'", actuator_info.name_.c_str());
  }

  // Register all joints
  for (transmission_interface::JointInfo joint_info : joint_infos)
  {
    // Create a new actuator
    joints_.push_back(Data(joint_info.name_));
    Data& joint = joints_.back();

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
  // TODO: Read the ethercat interface
  //
  //       1. Read the ethercat IO
  //       2. Convert the encoder counts to radians
  //       3. Write to the actuator state

  actuator_to_joint_transmission_interface_.propagate();
}

void SergioHardwareInterface::write(const ros::Time &, const ros::Duration &)
{
  joint_to_actuator_transmission_interface_.propagate();

  for (const Data& actuator : actuators_)
  {
    ROS_INFO("Sending %.3f [Nm] as command to actuator %s", actuator.command_, actuator.name_.c_str());
  }

  // TODO: Write the ethercat interface
  //
  //       1. Convert the effort to voltage
  //       2. Write to ethercat IO
}
