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
                     const std::vector<double>& output_default_values,
                     double publish_rate) : publish_rate_(publish_rate), output_names_(output_names)
{
  realtime_publisher_.init(ros::NodeHandle(), "io_states", 1);
  realtime_publisher_.msg_.io_states.resize(input_names.size());
  for (size_t i = 0; i < input_names.size(); ++i)
  {
    realtime_publisher_.msg_.io_states[i].key = input_names[i];
    realtime_publisher_.msg_.io_states[i].value = NAN;
  }
  ROS_INFO("IO Manager initialized, publish duration: %.2f", publish_rate_);

  if (output_names.size() != output_default_values.size())
  {
    throw std::runtime_error("Output names should be of same size as the output default values");
  }
  for (size_t i = 0; i < output_names.size(); ++i)
  {
    ROS_INFO("Configuring IO output %s with default value %.4f", output_names[i].c_str(), output_default_values[i]);
    output_data_[output_names[i]] = output_default_values[i];
  }

  ros::NodeHandle nh;
  output_srv_ = nh.advertiseService("set_io", &IOManager::outputSrv, this);
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

double* IOManager::getOutput(const std::string& name)
{
  if (output_data_.find(name) == output_data_.end())
  {
    throw std::runtime_error(name + " not found in outputs!");
  }
  return &output_data_[name];
}

void IOManager::updateOutputs()
{
  const control_msgs::IOState& last_received_io_state = *srv_buffer_.readFromRT();
  if (!last_received_io_state.key.empty())
  {
    output_data_[last_received_io_state.key] = last_received_io_state.value;
  }
}

void IOManager::publish(const ros::Time& time)
{
  // Throttle
  if (publish_rate_ > 0.0 && realtime_publisher_.msg_.header.stamp + ros::Duration(1.0 / publish_rate_) < time)
  {
    if (realtime_publisher_.trylock())
    {
      realtime_publisher_.msg_.header.stamp = time;
      realtime_publisher_.unlockAndPublish();
    }
  }
}

bool IOManager::outputSrv(control_msgs::SetIORequest& req, control_msgs::SetIOResponse &)
{
  if (std::find(output_names_.begin(), output_names_.end(), req.io_state.key) == output_names_.end())
  {
    std::stringstream ss;
    for (size_t i = 0; i < output_names_.size(); ++i)
    {
      ss << output_names_[i];
      if (i < output_names_.size() - 1)
      {
        ss << ", ";
      }
    }
    ROS_ERROR("Invalid io_state name %s, available output_names: [%s]", req.io_state.key.c_str(), ss.str().c_str());
    return false;
  }

  srv_buffer_.writeFromNonRT(req.io_state);
  return true;
}

}  // namespace io_manager
