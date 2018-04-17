//
// Copyright (c) 2018 TUe Robotics
//
// @author Rein Appeldoorn (reinzor)
//

#pragma once

#include <vector>
#include <string>
#include <iostream>
#include <ethercat_interface/ethercat_interface.h>

namespace ethercat_hardware_interface
{
struct EthercatInterfaceDescription
{
  size_t slave_ = 0;    // Ethercat slave id
  size_t channel_ = 0;  // Ethercat channel id
};
struct EthercatMotorInterfaceDescription : EthercatInterfaceDescription
{
  double volt_per_newton_meter_ = 0;  // Voltage to Nm mapping (determined by motor and amp)
};
inline std::ostream& operator<<(std::ostream& o, const EthercatMotorInterfaceDescription& a)
{
  o << "EthercatMotorInterfaceDescription(slave=" << a.slave_ << ", channel=" << a.channel_
    << ", volt_per_newton_meter=" << a.volt_per_newton_meter_ << ")";
  return o;
}

struct EthercatEncoderInterfaceDescription : EthercatInterfaceDescription
{
  size_t encoder_counts_per_revolution_;  // How many encoder counts for one revolution
};
inline std::ostream& operator<<(std::ostream& o, const EthercatEncoderInterfaceDescription& a)
{
  o << "EthercatEncoderInterfaceDescription(slave=" << a.slave_ << ", channel=" << a.channel_
    << ", encoder_counts_per_revolution=" << a.encoder_counts_per_revolution_ << ")";
  return o;
}

struct EthercatActuatorDescription
{
  EthercatMotorInterfaceDescription motor_;
  EthercatEncoderInterfaceDescription encoder_;
};
inline std::ostream& operator<<(std::ostream& o, const EthercatActuatorDescription& a)
{
  o << "EthercatActuatorDescription(motor=" << a.motor_ << ", encoder=" << a.encoder_ << ")";
  return o;
}
}  // namespace ethercat_hardware_interface
