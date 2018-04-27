//
// Copyright (c) 2018 TUe Robotics
//
// @author Rein Appeldoorn (reinzor)
//

#pragma once

#include <vector>
#include <string>
#include <map>
#include <memory>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include <control_msgs/IOStates.h>
#include <control_msgs/SetIO.h>

namespace io_manager
{
class IOManager
{
public:

  //!
  //! \brief IOManager
  //! \param ros_control_interfaces
  //!
  explicit IOManager(const std::vector<std::string>& input_names,
                     const std::vector<std::string>& output_names,
                     const std::vector<double>& output_default_values,
                     double publish_rate);

  double* getInput(const std::string& name);
  double* getOutput(const std::string& name);

  void publish(const ros::Time& time);

  void updateOutputs();

private:
  double publish_rate_;
  realtime_tools::RealtimePublisher<control_msgs::IOStates> realtime_publisher_;

  std::vector<std::string> output_names_;
  std::map<std::string, double> output_data_;
  realtime_tools::RealtimeBuffer<control_msgs::IOState> srv_buffer_;

  ros::ServiceServer output_srv_;
  bool outputSrv(control_msgs::SetIORequest& req, control_msgs::SetIOResponse&);

};
}  // namespace io_manager
