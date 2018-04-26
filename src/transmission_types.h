//
// Copyright (c) 2018 TUe Robotics
//
// @author Rein Appeldoorn (reinzor)
//

#pragma once

#include <string>
#include <ostream>
#include <memory>

#include <joint_limits_interface/joint_limits.h>

struct JointState
{
  // state
  double raw_position_ = 0;
  double position_offset_ = 0;    // calibration
  double calibrated_position_ = 0;
  double velocity_ = 0;
  double effort_ = 0;

  // reference
  double command_ = 0;

  std::string name_;

  explicit JointState(const std::string& name) : name_(name)
  {
  }
};
inline std::ostream& operator<<(std::ostream& o, const JointState& s)
{
  o << "JointState(name=" << s.name_ << ", raw_position=" << s.raw_position_
    << ", position_offset=" << s.position_offset_ << ", calibrated_position=" << s.calibrated_position_
    << ", velocity=" << s.velocity_ << ", effort=" << s.effort_ << ", command=" << s.command_ << ")";
  return o;
}
typedef std::shared_ptr<JointState> JointStatePtr;

struct ActuatorState
{
  // state
  double position_ = 0;
  double velocity_ = 0;
  double effort_ = 0;

  // reference
  double command_ = 0;

  std::string name_;

  explicit ActuatorState(const std::string& name) : name_(name)
  {
  }
};
inline std::ostream& operator<<(std::ostream& o, const ActuatorState& s)
{
  o << "ActuatorState(name=" << s.name_ << ", position=" << s.position_ << ", velocity=" << s.velocity_
    << ", effort=" << s.effort_ << ", command=" << s.command_ << ")";
  return o;
}
typedef std::shared_ptr<ActuatorState> ActuatorStatePtr;
