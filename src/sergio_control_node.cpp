#include <string>

#include "controller_manager/controller_manager.h"
#include "sergio_hardware_interface.h"
#include "ros/ros.h"


//!
//! \brief controlThread Separate thread for running the controller
//!
void controlThread(ros::Rate rate, SergioHardwareInterface* robot, controller_manager::ControllerManager* cm)
{
  ros::Time last_cycle_time = ros::Time::now();
  while (ros::ok())
  {
    robot->read(ros::Time::now(), ros::Time::now() - last_cycle_time);
    cm->update(ros::Time::now(), ros::Time::now() - last_cycle_time);
    robot->write(ros::Time::now(), ros::Time::now() - last_cycle_time);

    if (rate.cycleTime() > rate.expectedCycleTime())
    {
      ROS_WARN_STREAM_DELAYED_THROTTLE(10.0, "Cycle time too high: Cycle time: " << rate.cycleTime() <<
                                       ", Expected cycle time: " << rate.expectedCycleTime());
    }

    last_cycle_time = ros::Time::now();

    rate.sleep();
  }
}


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "sergio_control");

  ros::NodeHandle local_nh("~");
  ros::Rate rate(local_nh.param("frequency", 1000));
  std::string ethernet_interface = local_nh.param("ethernet_interface", std::string("eth0"));

  try
  {
    SergioHardwareInterface robot(ethernet_interface);

    controller_manager::ControllerManager cm(&robot);

    boost::thread(boost::bind(controlThread, rate, &robot, &cm));

    ROS_INFO("Sergio Controller Manager initialized, spinning ...");

    ros::spin();
  }
  catch (const std::exception& e)
  {
    ROS_FATAL_STREAM("Sergio control error: " << e.what());
    return 1;
  }

  return 0;
}
