#pragma once

#include <ros/console.h>

#include "./ethercat_interface_descriptions.h"
#include "./transmission_types.h"

class EthercatActuator
{
public:
  EthercatActuator(const EthercatActuatorDescription& description, ActuatorState* state, EthercatInterface& interface)
    : state_(state), description_(description)
  {
    ROS_INFO("Registering analogue out interface on slave %d and channel %d...", (int)description.motor_.slave_,
             (int)description.motor_.channel_);
    analogue_out_ = interface.getInterface(description.motor_.slave_, description.motor_.channel_);
    ROS_INFO("Registering digital in interface on slave %d and channel %d ...", (int)description.encoder_.slave_,
             (int)description.encoder_.channel_);
    encoder_in_ = interface.getInterface(description.encoder_.slave_, description.encoder_.channel_);
    ROS_INFO("Actuator initialized");
  }

  EthercatActuatorDescription description_;
  ActuatorState* state_;

  bool write()
  {
    double voltage = state_->command_ * description_.motor_.volt_per_newton_meter_;
    ROS_INFO("Sending %.5f [Nm] %.5f [V] as command to actuator %s", state_->command_, voltage, state_->name_.c_str());
    return analogue_out_->write(voltage);
  }

  void read()
  {
    int encoder_value = encoder_in_->read();
    state_->position_ = (double)encoder_value / description_.encoder_.encoder_counts_per_revolution_ * 2 * M_PI;
    ROS_INFO("Read actuator encoder value: %d, position: %.2f from actuator %s",
             encoder_value, state_->position_, state_->name_.c_str());
  }

private:
  std::shared_ptr<IOInterface> analogue_out_;
  std::shared_ptr<IOInterface> encoder_in_;
};
