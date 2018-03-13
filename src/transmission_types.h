#pragma once

#include <string>
#include <ostream>

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
inline std::ostream& operator << (std::ostream &o, const JointState &s)
{
  o << "JointState(name=" << s.name_ << ", position=" << s.position_ << ", velocity=" << s.velocity_ << ", effort="
    << s.effort_ << ", command=" << s.command_ << ")";
  return o;
}

struct ActuatorState
{
  // state
  double position_ = 0;
  double velocity_ = 0;
  double effort_ = 0;

  // reference
  double command_ = 0;

  std::string name_;

  ActuatorState(const std::string& name) : name_(name)
  {

  }
};
inline std::ostream& operator << (std::ostream &o, const ActuatorState &s)
{
  o << "ActuatorState(name=" << s.name_ << ", position=" << s.position_ << ", velocity=" << s.velocity_ << ", effort="
    << s.effort_ << ", command=" << s.command_ << ")";
  return o;
}
