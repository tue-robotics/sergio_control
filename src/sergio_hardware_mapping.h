#ifndef SERGIO_HARDWARE_MAPPING_H
#define SERGIO_HARDWARE_MAPPING_H

#include <vector>
#include <string>
#include <iostream>

namespace sergio_control
{

struct EthercatInterfaceDescription
{
  size_t slave_ = 0; // Ethercat slave id
  size_t channel_ = 0; // Ethercat channel id
};
struct EthercatMotorInterfaceDescription : EthercatInterfaceDescription
{
  double volt_per_newton_meter_ = 0; // Voltage to Nm mapping (determined by motor and amp)
};
inline std::ostream& operator << (std::ostream &o, const EthercatMotorInterfaceDescription &a)
{
  o << "EthercatMotorInterfaceDescription(slave=" << a.slave_ << ", channel=" << a.channel_
    << ", volt_per_newton_meter=" << a.volt_per_newton_meter_ << ")";
  return o;
}

struct EthercatEncoderInterfaceDescription : EthercatInterfaceDescription
{
  size_t encoder_counts_per_revolution_; // How many encoder counts for one revolution
};
inline std::ostream& operator << (std::ostream &o, const EthercatEncoderInterfaceDescription &a)
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

#include <ethercat_interface/ethercat_interface.h>

struct Actuator
{
  Actuator(const std::string& name,
           EthercatActuatorDescription description,
           EthercatInterface& interface) :
    data_(name),
    description_(description),
    analogue_out_(interface.getInterface(description.motor_.slave_, description.motor_.channel_))
  {
  }

  EthercatActuatorDescription description_;
  std::shared_ptr<IOInterface> analogue_out_;
  std::shared_ptr<IOInterface> digital_in_;

  JointState data_;
};

}

#endif
