//
// Copyright (c) 2018 TUe Robotics
//
// @author Rein Appeldoorn (reinzor)
//

#pragma once

#include <xmlrpcpp/XmlRpc.h>
#include <ros/console.h>
#include <map>
#include <string>

#include "./ethercat_interface_descriptions.h"

namespace ethercat_hardware_interface
{
//!
//! \brief getSlaveAndChannel Return the slave and channel from a XMLRPC struct
//! \param param The struct xml rpc
//! \param slave Slave id reference
//! \param channel Channel id reference
//!
inline void getSlaveAndChannel(XmlRpc::XmlRpcValue param, size_t* slave, size_t* channel)
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

  *slave = static_cast<int>(slave_xmlrpc);
  *channel = static_cast<int>(channel_xmlrpc);
}

//!
//! \brief getEthercatMotorInterfaceDescription Returns the motor interface description
//! \param param XML RPC param of the motor
//! \return Description
//!
inline EthercatMotorInterfaceDescription getEthercatMotorInterfaceDescription(XmlRpc::XmlRpcValue param)
{
  EthercatMotorInterfaceDescription description;
  getSlaveAndChannel(param, &description.slave_, &description.channel_);

  if (!param.hasMember("scale_factor"))
  {
    throw std::runtime_error("An EthercatMotorInterfaceDescription should have a scale_factor key");
  }
  XmlRpc::XmlRpcValue scale_factor_xmlpc = param["scale_factor"];
  if (scale_factor_xmlpc.getType() != XmlRpc::XmlRpcValue::TypeDouble)
  {
    throw std::runtime_error("Scale factor should be a double");
  }
  description.scale_factor_ = static_cast<double>(scale_factor_xmlpc);

  return description;
}

//!
//! \brief getEthercatEncoderInterfaceDescription Returns the encoder interface description
//! \param param XML RPC param of the encoder
//! \return Description
//!
inline EthercatEncoderInterfaceDescription getEthercatEncoderInterfaceDescription(XmlRpc::XmlRpcValue param)
{
  EthercatEncoderInterfaceDescription description;
  getSlaveAndChannel(param, &description.slave_, &description.channel_);

  if (!param.hasMember("encoder_counts_per_revolution"))
  {
    throw std::runtime_error("An EthercatEncoderInterfaceDescription should have a encoder_counts_per_revolution key");
  }
  XmlRpc::XmlRpcValue encoder_counts_per_revolution_xmlrpc = param["encoder_counts_per_revolution"];
  if (encoder_counts_per_revolution_xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeInt)
  {
    throw std::runtime_error("encoder_counts_per_revolution should be a int");
  }
  description.encoder_counts_per_revolution_ = static_cast<int>(encoder_counts_per_revolution_xmlrpc);

  return description;
}

inline EthercatJointPositionInterfaceDescription getEthercatJointPositionInterfaceDescription(XmlRpc::XmlRpcValue param)
{
  EthercatJointPositionInterfaceDescription description;
  getSlaveAndChannel(param, &description.slave_, &description.channel_);

  if (!param.hasMember("scale_factor"))
  {
    throw std::runtime_error("An EthercatJointPositionInterfaceDescription should have a scale_factor key");
  }
  XmlRpc::XmlRpcValue scale_factor_xmlrpc = param["scale_factor"];
  if (scale_factor_xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeDouble)
  {
    throw std::runtime_error("scale_factor should be a double");
  }
  description.scale_factor_ = static_cast<double>(scale_factor_xmlrpc);

  if (!param.hasMember("offset"))
  {
    throw std::runtime_error("An EthercatJointPositionInterfaceDescription should have a offset key");
  }
  XmlRpc::XmlRpcValue offset_xmlrpc = param["offset"];
  if (scale_factor_xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeDouble)
  {
    throw std::runtime_error("offset should be a double");
  }
  description.offset_ = static_cast<double>(offset_xmlrpc);

  return description;
}

//!
//! \brief getEthercatActuatorsDescription Parse the actuators description from the parameter server
//! \param param The XML RPC parameter
//! \return Return a map from actuator names to descriptions
//!
inline std::map<std::string, EthercatActuatorInterfaceDescription> getEthercatActuatorInterfacesDescription(XmlRpc::XmlRpcValue param)
{
  std::map<std::string, EthercatActuatorInterfaceDescription> ethercat_actuators_description;
  for (XmlRpc::XmlRpcValue::ValueStruct::const_iterator act_it = param.begin(); act_it != param.end(); ++act_it)
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

    EthercatActuatorInterfaceDescription description;
    description.encoder_ = getEthercatEncoderInterfaceDescription(actuator_description_xmlrpc["encoder"]);
    description.motor_ = getEthercatMotorInterfaceDescription(actuator_description_xmlrpc["motor"]);
    ethercat_actuators_description[actuator_name] = description;

    ROS_INFO_STREAM("Parsed " << actuator_name << ": " << description);
  }

  if (ethercat_actuators_description.empty())
  {
    ROS_WARN("No ethercat actuators found");
  }

  return ethercat_actuators_description;
}

inline std::map<std::string, EthercatJointPositionInterfaceDescription> getEthercatJointPositionInterfacesDescription(XmlRpc::XmlRpcValue param)
{
  std::map<std::string, EthercatJointPositionInterfaceDescription> ethercat_joint_position_interfaces;
  for (XmlRpc::XmlRpcValue::ValueStruct::const_iterator act_it = param.begin(); act_it != param.end(); ++act_it)
  {
    XmlRpc::XmlRpcValue joint_name_xmlrpc = act_it->first;
    XmlRpc::XmlRpcValue joint_position_interface_xmlrpc = act_it->second;
    if (joint_name_xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeString)
    {
      throw std::runtime_error("Key should be of type string");
    }
    if (joint_position_interface_xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      throw std::runtime_error("Value should be of type struct");
    }
    std::string joint_name = static_cast<std::string>(joint_name_xmlrpc);
    ROS_DEBUG("Parsing joint name %s", joint_name.c_str());

    ethercat_joint_position_interfaces[joint_name] = getEthercatJointPositionInterfaceDescription(joint_position_interface_xmlrpc);

    ROS_INFO_STREAM("Parsed " << joint_name << ": " <<  ethercat_joint_position_interfaces[joint_name]);
  }

  if (ethercat_joint_position_interfaces.empty())
  {
    ROS_WARN("No ethercat_joint_position_interfaces found");
  }

  return ethercat_joint_position_interfaces;
}

inline std::vector<EthercatInterfaceDescription> getEthercatInterfaceDescription(XmlRpc::XmlRpcValue param)
{
  std::vector<EthercatInterfaceDescription> ethercat_interface_description;
  for (XmlRpc::XmlRpcValue::ValueStruct::const_iterator act_it = param.begin(); act_it != param.end(); ++act_it)
  {
    XmlRpc::XmlRpcValue io_name_xmlrpc = act_it->first;
    XmlRpc::XmlRpcValue io_xmlrpc = act_it->second;
    if (io_name_xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeString)
    {
      throw std::runtime_error("Key should be of type string");
    }
    if (io_xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      throw std::runtime_error("Value should be of type struct");
    }

    EthercatInterfaceDescription description;
    description.name_ = static_cast<std::string>(io_name_xmlrpc);
    getSlaveAndChannel(io_xmlrpc, &description.slave_, &description.channel_);
    ethercat_interface_description.push_back(description);

    ROS_INFO_STREAM("Parsed " << description);
  }

  return ethercat_interface_description;
}

}  // namespace ethercat_hardware_interface
