#ifndef ACTUATOR_PARSER_H
#define ACTUATOR_PARSER_H

#include "sergio_hardware_mapping.h"
#include <transmission_interface/transmission_parser.h>

namespace sergio_control
{

//!
//! \brief getSlaveAndChannel Return the slave and channel from a XMLRPC struct
//! \param param The struct xml rpc
//! \param slave Slave id reference
//! \param channel Channel id reference
//!
inline void getSlaveAndChannel(XmlRpc::XmlRpcValue param, size_t& slave, size_t& channel)
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

  slave = static_cast<int>(slave_xmlrpc);
  channel = static_cast<int>(channel_xmlrpc);
}

//!
//! \brief getEthercatMotorInterfaceDescription Returns the motor interface description
//! \param param XML RPC param of the motor
//! \return Description
//!
inline EthercatMotorInterfaceDescription getEthercatMotorInterfaceDescription(XmlRpc::XmlRpcValue param)
{
  EthercatMotorInterfaceDescription des;
  getSlaveAndChannel(param, des.slave_, des.channel_);

  if (!param.hasMember("volt_per_newton_meter"))
  {
    throw std::runtime_error("An EthercatMotorInterfaceDescription should have a volt_per_newton_meter key");
  }
  XmlRpc::XmlRpcValue volt_per_newton_meter_xmlpc = param["volt_per_newton_meter"];
  if (volt_per_newton_meter_xmlpc.getType() != XmlRpc::XmlRpcValue::TypeDouble)
  {
    throw std::runtime_error("Volt per newton meter should be a double");
  }
  des.volt_per_newton_meter_ = static_cast<double>(volt_per_newton_meter_xmlpc);

  return des;
}

//!
//! \brief getEthercatEncoderInterfaceDescription Returns the encoder interface description
//! \param param XML RPC param of the encoder
//! \return Description
//!
inline EthercatEncoderInterfaceDescription getEthercatEncoderInterfaceDescription(XmlRpc::XmlRpcValue param)
{
  EthercatEncoderInterfaceDescription des;
  getSlaveAndChannel(param, des.slave_, des.channel_);

  if (!param.hasMember("encoder_counts_per_revolution"))
  {
    throw std::runtime_error("An EthercatEncoderInterfaceDescription should have a encoder_counts_per_revolution key");
  }
  XmlRpc::XmlRpcValue encoder_counts_per_revolution_xmlrpc = param["encoder_counts_per_revolution"];
  if (encoder_counts_per_revolution_xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeInt)
  {
    throw std::runtime_error("Volt per newton meter should be a double");
  }
  des.encoder_counts_per_revolution_ = static_cast<int>(encoder_counts_per_revolution_xmlrpc);

  return des;
}

//!
//! \brief getEthercatActuatorsDescription Parse the actuators description from the parameter server
//! \param param The XML RPC parameter
//! \return Return a map from actuator names to descriptions
//!
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
    description.encoder_ = getEthercatEncoderInterfaceDescription(actuator_description_xmlrpc["encoder"]);
    description.motor_ = getEthercatMotorInterfaceDescription(actuator_description_xmlrpc["motor"]);
    ethercat_actuators_description[actuator_name] = description;

    ROS_INFO_STREAM("Parsed " << actuator_name << ": " << description);
  }

  if (ethercat_actuators_description.empty())
  {
    throw std::runtime_error("No ethercat actuators found");
  }

  return ethercat_actuators_description;
}

//!
//! \brief getActuator Get an actuator from an actuator description
//! \param actuator_info The actuator information from the URDF
//! \param ethercat_actuators_description The available ethercat actuators descripton
//! \param ethercat_interface Reference to ethercat interface
//! \return An actuator that holds the state, a reference and an interface to ethercat
//!
inline Actuator getActuator(const transmission_interface::ActuatorInfo& actuator_info,
                            const std::map<std::string, EthercatActuatorDescription>& ethercat_actuators_description,
                            EthercatInterface& ethercat_interface)
{
  ROS_INFO("Getting actuator %s from ethercat actuators description ...", actuator_info.name_.c_str());

  // Lookup the actuator in the ethercat actuators description
  std::map<std::string, EthercatActuatorDescription>::const_iterator ethercat_actuator =
      ethercat_actuators_description.find(actuator_info.name_);

  if (ethercat_actuator == ethercat_actuators_description.end())
  {
    throw std::runtime_error(actuator_info.name_ + " could not be found in the ethercat actuators description");
  }

  return Actuator(actuator_info.name_, ethercat_actuator->second, ethercat_interface);
}

}

#endif
