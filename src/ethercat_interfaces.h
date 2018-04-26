//
// Copyright (c) 2018 TUe Robotics
//
// @author Rein Appeldoorn (reinzor)
//

#pragma once

#include <ros/console.h>

#include "./ethercat_interface_descriptions.h"
#include "./transmission_types.h"

namespace ethercat_hardware_interface
{
class EthercatActuator
{
public:
  EthercatActuator(const EthercatActuatorInterfaceDescription& description, ethercat_interface::InterfacePtr interface,
                   std::shared_ptr<ActuatorState> state)
    : state_(state), description_(description)
  {
    ROS_INFO("Registering analogue out interface on slave %d and channel %d...", (int)description.motor_.slave_,
             (int)description.motor_.channel_);
    analogue_out_ = interface->getSlave(description.motor_.slave_).getOutput(description.motor_.channel_);
    ROS_INFO("Registering digital in interface on slave %d and channel %d ...", (int)description.encoder_.slave_,
             (int)description.encoder_.channel_);
    encoder_in_ = interface->getSlave(description.encoder_.slave_).getInput(description.encoder_.channel_);
    ROS_INFO("Actuator initialized");
  }

  ActuatorStatePtr state_;
  EthercatActuatorInterfaceDescription description_;

  bool write()
  {
    double output = state_->command_ * description_.motor_.scale_factor_;
    ROS_DEBUG("Sending %.5f [Nm] that converts to %.5f [Output specific] as command to actuator %s",
              state_->command_, output, state_->name_.c_str());
    return analogue_out_->write(output);
  }

  void read(const ros::Duration& period)
  {
    // Store last position for velocity calculation
    double last_position = state_->position_;

    int encoder_value = encoder_in_->read();
    state_->position_ =
        static_cast<double>(encoder_value) / description_.encoder_.encoder_counts_per_revolution_ * 2 * M_PI;
    if (period.toSec() >= 0)
    {
      state_->velocity_ = (state_->position_ - last_position) / period.toSec();
    }
    ROS_DEBUG("Read actuator encoder value: %d, position: %.5f, velocity: %.5f from actuator %s", encoder_value,
              state_->position_, state_->velocity_, state_->name_.c_str());
  }

private:
  ethercat_interface::OutputPtr analogue_out_;
  ethercat_interface::InputPtr encoder_in_;
};

class EthercatJointPositionInterface
{
public:
  EthercatJointPositionInterface(const EthercatJointPositionInterfaceDescription& description,
                                 ethercat_interface::InterfacePtr interface, std::shared_ptr<JointState> state)
    : description_(description), state_(state)
  {
    ROS_INFO("Registering position in interface on slave %zu and channel %zu ...",
             description.slave_, description.channel_);
    position_in_ = interface->getSlave(description.slave_).getInput(description.channel_);
    ROS_INFO("EthercatJointPositionInterface initialized");
  }

  void read()
  {
    double value = position_in_->read();
    state_->calibrated_position_ = value * description_.scale_factor_ + description_.offset_;
    ROS_DEBUG("Read raw value: %.4f, joint position value: %.4f", value, state_->calibrated_position_);
  }

  EthercatJointPositionInterfaceDescription description_;
  JointStatePtr state_;

private:
  ethercat_interface::InputPtr position_in_;
};

}  // namespace ethercat_hardware_interface
