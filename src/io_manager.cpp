//
// Copyright (c) 2018 TUe Robotics
//
// @author Rein Appeldoorn (reinzor)
//

#include "./io_manager.h"

namespace io_manager
{

IOManager::IOManager(const std::vector<std::string>& input_names,
                     const std::vector<std::string>& output_names,
                     double publish_rate) : publish_rate_(publish_rate)
{
  realtime_publisher_.init(ros::NodeHandle(), "io_states", 1);
  realtime_publisher_.msg_.io_states.resize(input_names.size());
  for (unsigned int i = 0; i < input_names.size(); ++i)
  {
    realtime_publisher_.msg_.io_states[i].key = input_names[i];
    realtime_publisher_.msg_.io_states[i].value = NAN;
  }
  ROS_INFO("IO Manager initialized, publish duration: %.2f", publish_rate_);
}

double* IOManager::getInput(const std::string& name)
{
  for (auto& io_state : realtime_publisher_.msg_.io_states)
  {
    if (io_state.key == name)
    {
      return &io_state.value;
    }
  }
  throw std::runtime_error(name + " not found in inputs!");
}

void IOManager::publish(const ros::Time& time)
{
  // Throttle
  if (publish_rate_ > 0.0 && ealtime_publisher_.msg_.header.stamp + ros::Duration(1.0 / publish_rate_) < time)
  {
    if (realtime_publisher_.trylock())
    {
      realtime_publisher_.msg_.header.stamp = time;
      realtime_publisher_.unlockAndPublish();
    }
  }
}

}  // namespace io_manager
