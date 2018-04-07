#pragma once

#include <ros/console.h>

#include "./ethercat_interface_descriptions.h"
#include "./transmission_types.h"

namespace ethercat_hardware_interface
{
class EthercatActuator
{
public:
  EthercatActuator(const EthercatActuatorDescription& description, ActuatorState* state, EthercatInterface& interface)
    : state_(state), description_(description)
  {
    ROS_INFO("Registering analogue out interface on slave %d and channel %d...", (int)description.motor_.slave_,
             (int)description.motor_.channel_);
    analogue_out_ = interface.getSlave(description.motor_.slave_).getOutput(description.motor_.channel_);
    ROS_INFO("Registering digital in interface on slave %d and channel %d ...", (int)description.encoder_.slave_,
             (int)description.encoder_.channel_);
    encoder_in_ = interface.getSlave(description.encoder_.slave_).getInput(description.encoder_.channel_);
    ROS_INFO("Actuator initialized");
  }

  EthercatActuatorDescription description_;
  ActuatorState* state_;

  bool write()
  {
    double voltage = state_->command_ * description_.motor_.volt_per_newton_meter_;
    ROS_DEBUG("Sending %.5f [Nm] %.5f [V] as command to actuator %s", state_->command_, voltage, state_->name_.c_str());
    return analogue_out_->write(voltage);
  }

  void read(const ros::Duration& period)
  {
    // Store last position for velocity calculation
    double last_position = state_->position_;

    int encoder_value = encoder_in_->read();
    state_->position_ = (double)encoder_value / description_.encoder_.encoder_counts_per_revolution_ * 2 * M_PI;
    state_->velocity_ = (state_->position_ - last_position) / period.toSec();
    ROS_DEBUG("Read actuator encoder value: %d, position: %.5f, velocity: %.5f from actuator %s",
              encoder_value, state_->position_, state_->velocity_, state_->name_.c_str());
  }

private:
  std::shared_ptr<WriteInterface> analogue_out_;
  std::shared_ptr<ReadInterface> encoder_in_;
};
}
