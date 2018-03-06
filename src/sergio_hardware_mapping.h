#ifndef SERGIO_HARDWARE_MAPPING_H
#define SERGIO_HARDWARE_MAPPING_H

#include <vector>
#include <string>
#include <iostream>

namespace sergio_control
{

const size_t ENCODER_COUNTS_PER_CYCLE = 256;

struct EthercatInterfaceDescription
{
  size_t slave_ = 0;
  size_t channel_ = 0;
};
inline std::ostream& operator << (std::ostream &o, const EthercatInterfaceDescription &a)
{
  o << "EthercatInterfaceDescription(slave=" << a.slave_ << ", channel=" << a.channel_ << ")";
  return o;
}

struct EthercatActuatorDescription
{
  EthercatInterfaceDescription motor_;
  EthercatInterfaceDescription encoder_;
};
inline std::ostream& operator << (std::ostream &o, const EthercatActuatorDescription &a)
{
  o << "EthercatActuatorDescription(motor=" << a.motor_ << ", encoder=" << a.encoder_ << ")";
  return o;
}

struct JointState
{
  // state
  double position_ = 0;
  double velocity_ = 0;
  double effort_ = 0;

  // reference
  double command_ = 0;

  std::string name_;

  JointState(const std::string& name) : name_(name)
  {

  }
};

struct Actuator
{
  Actuator(const std::string& name, const EthercatActuatorDescription& description) : data_(name)
  {
    // TODO Create pointer to encoder and motor
  }

  JointState data_;
};

}

#endif
