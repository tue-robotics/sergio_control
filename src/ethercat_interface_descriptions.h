//
// Copyright (c) 2018 TUe Robotics
//
// @author Rein Appeldoorn (reinzor)
//

#pragma once

#include <vector>
#include <string>
#include <iostream>
#include <ethercat_interface/interface.h>

namespace ethercat_hardware_interface
{

struct EthercatInterfaceDescription
{
  std::string name_ = "";
  size_t slave_ = 0;    // Ethercat slave id
  size_t channel_ = 0;  // Ethercat channel id
};
inline std::ostream& operator<<(std::ostream& o, const EthercatInterfaceDescription& a)
{
  o << "EthercatInterfaceDescription(slave=" << a.slave_ << ", channel=" << a.channel_ << ")";
  return o;
}

struct EthercatOutputInterfaceDescription : EthercatInterfaceDescription
{
  double default_value_ = 0;
};
inline std::ostream& operator<<(std::ostream& o, const EthercatOutputInterfaceDescription& a)
{
  o << "EthercatOutputInterfaceDescription(slave=" << a.slave_ << ", channel=" << a.channel_
    << ", default_value=" << a.default_value_ << ")";
  return o;
}

struct EthercatMotorInterfaceDescription : EthercatInterfaceDescription
{
  double scale_factor_ = 0;  // Output to Nm mapping (determined by motor, amplifier or current control)
};
inline std::ostream& operator<<(std::ostream& o, const EthercatMotorInterfaceDescription& a)
{
  o << "EthercatMotorInterfaceDescription(slave=" << a.slave_ << ", channel=" << a.channel_
    << ", scale_factor=" << a.scale_factor_ << ")";
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

struct EthercatEnableInterfaceDescription : EthercatInterfaceDescription
{
  bool need_enable_ = true;
};
inline std::ostream& operator<<(std::ostream& o, const EthercatEnableInterfaceDescription& a)
{
    o << "EthercatEnableInterfaceDescription(slave=" << a.slave_ << ", channel=" << a.channel_ << ", enable=" << a.need_enable_ << ")";
  return o;
}

struct EthercatActuatorInterfaceDescription
{
  EthercatMotorInterfaceDescription motor_;
  EthercatEncoderInterfaceDescription encoder_;
  EthercatEnableInterfaceDescription enable_;
};
inline std::ostream& operator<<(std::ostream& o, const EthercatActuatorInterfaceDescription& a)
{
  o << "EthercatActuatorInterfaceDescription(motor=" << a.motor_ << ", encoder=" << a.encoder_ << ", enable=" << a.enable_ << ")";
  return o;
}

struct EthercatJointPositionInterfaceDescription : EthercatInterfaceDescription
{
  double offset_ = 0;  // Offset at zero
  double scale_factor_ = 0;  // Scale factor from measurement to SI [radians or m]
};
inline std::ostream& operator<<(std::ostream& o, const EthercatJointPositionInterfaceDescription& a)
{
  o << "EthercatJointPositionInterfaceDescription(slave=" << a.slave_ << ", channel=" << a.channel_
    << ", scale_factor=" << a.scale_factor_ << ", offset=" << a.offset_ << ")";
  return o;
}

}  // namespace ethercat_hardware_interface
