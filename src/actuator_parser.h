#ifndef ACTUATOR_PARSER_H
#define ACTUATOR_PARSER_H

#include "sergio_hardware_mapping.h"
#include <transmission_interface/transmission_parser.h>

namespace sergio_control
{

inline EthercatInterfaceDescription getEthercatInterfaceDescription(XmlRpc::XmlRpcValue param)
{
  if (param.getType() != XmlRpc::XmlRpcValue::TypeStruct)
  {
    throw std::runtime_error("EthercatInterfaceDescription should be of type struct");
  }
  if (!param.hasMember("slave") || !param.hasMember("channel"))
  {
    throw std::runtime_error("An EthercatInterfaceDescription should have a slave and channel key");
  }

  XmlRpc::XmlRpcValue slave_xmlrpc = param["slave"];
  XmlRpc::XmlRpcValue channel_xmlrpc = param["channel"];
  if (slave_xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeInt ||
      channel_xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeInt)
  {
    throw std::runtime_error("Channel and slave should be of type int");
  }

  EthercatInterfaceDescription description;
  description.slave_ = static_cast<int>(slave_xmlrpc);
  description.channel_ = static_cast<int>(channel_xmlrpc);
  return description;
}

inline std::map<std::string, EthercatActuatorDescription> getEthercatActuatorsDescription(XmlRpc::XmlRpcValue param)
{
  std::map<std::string, EthercatActuatorDescription> ethercat_actuators_description;
  for(XmlRpc::XmlRpcValue::ValueStruct::const_iterator act_it = param.begin(); act_it != param.end(); ++act_it)
  {
    XmlRpc::XmlRpcValue actuator_name_xmlrpc = act_it->first;
    XmlRpc::XmlRpcValue actuator_description_xmlrpc = act_it->second;
    if (actuator_name_xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeString)
    {
      throw std::runtime_error("Key should be of type string");
    }
    if (actuator_description_xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      throw std::runtime_error("Value should be of type struct");
    }
    std::string actuator_name = static_cast<std::string>(actuator_name_xmlrpc);
    ROS_DEBUG("Parsing actuator %s", actuator_name.c_str());

    // Check if the description has all required values
    if (!actuator_description_xmlrpc.hasMember("motor") || !actuator_description_xmlrpc.hasMember("encoder"))
    {
      throw std::runtime_error("An actuator should have a motor and encoder key");
    }

    EthercatActuatorDescription description;
    description.encoder_ = getEthercatInterfaceDescription(actuator_description_xmlrpc["encoder"]);
    description.motor_ = getEthercatInterfaceDescription(actuator_description_xmlrpc["motor"]);
    ethercat_actuators_description[actuator_name] = description;

    ROS_INFO_STREAM("Parsed " << actuator_name << ": " << description);
  }

  if (ethercat_actuators_description.empty())
  {
    throw std::runtime_error("No ethercat actuators found");
  }

  return ethercat_actuators_description;
}

inline Actuator getActuator(const transmission_interface::ActuatorInfo& actuator_info,
                            const std::map<std::string, EthercatActuatorDescription>& ethercat_actuators_description)
{
  ROS_INFO("Getting actuator %s from ethercat actuators description ...", actuator_info.name_.c_str());

  // Lookup the actuator in the ethercat actuators description
  std::map<std::string, EthercatActuatorDescription>::const_iterator ethercat_actuator =
      ethercat_actuators_description.find(actuator_info.name_);

  if (ethercat_actuator == ethercat_actuators_description.end())
  {
    throw std::runtime_error(actuator_info.name_ + " could not be found in the ethercat actuators description");
  }

  // TODO obtain
  return Actuator(actuator_info.name_, ethercat_actuator->second);
}

}

#endif
