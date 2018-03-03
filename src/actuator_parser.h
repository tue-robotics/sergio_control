#ifndef ACTUATOR_PARSER_H
#define ACTUATOR_PARSER_H

#include "sergio_hardware_mapping.h"
#include <transmission_interface/transmission_parser.h>

static size_t getIntFromXML(const std::string& xml, const std::string element_name)
{
//  TiXmlDocument doc;
//  if (!doc.Parse(xml.c_str()))
//  {
//    throw std::runtime_error("getIntFromXML: could not parse xml " + xml);
//  }
  //! TODO, Y u no work?
  return boost::lexical_cast<int>("1");
}

static Actuator getActuator(const transmission_interface::ActuatorInfo& actuator_info)
{
  ROS_INFO("Getting actuator %s from xml ...", actuator_info.name_.c_str());
  return Actuator(actuator_info.name_,
                  getIntFromXML(actuator_info.xml_element_, "inputSlave"),
                  getIntFromXML(actuator_info.xml_element_, "inputChannel"),
                  getIntFromXML(actuator_info.xml_element_, "outputSlave"),
                  getIntFromXML(actuator_info.xml_element_, "outputChannel"));
}

#endif
