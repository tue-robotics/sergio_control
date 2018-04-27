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
                     double publish_rate);

  double* getInput(const std::string& name);

  void publish(const ros::Time& time);

private:
  double publish_rate_;
  realtime_tools::RealtimePublisher<control_msgs::IOStates> realtime_publisher_;

};
}  // namespace io_manager
