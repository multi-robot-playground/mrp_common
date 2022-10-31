#ifndef MULTI_ROBOT_PLAYGROUND_COMMON__LOGGING_HPP_
#define MULTI_ROBOT_PLAYGROUND_COMMON__LOGGING_HPP_

#include "rclcpp/rclcpp.hpp"

namespace mrp_common
{
  class Log
  {
  public:
    static void basicDebug(
        rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface,
        const std::string &msg)
    {
      RCLCPP_DEBUG(node_logging_interface->get_logger(),"%s", msg.c_str());
    }

    static void basicInfo(
        rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface,
        const std::string &msg)
    {
      RCLCPP_INFO(node_logging_interface->get_logger(),"%s", msg.c_str());
    }

    static void basicWarn(
        rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface,
        const std::string &msg)
    {
      RCLCPP_WARN(node_logging_interface->get_logger(),"%s", msg.c_str());
    }
    
    static void basicError(
        rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface,
        const std::string &msg)
    {
      RCLCPP_ERROR(node_logging_interface->get_logger(),"%s", msg.c_str());
    }

    static void basicFatal(
        rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface,
        const std::string &msg)
    {
      RCLCPP_FATAL(node_logging_interface->get_logger(),"%s", msg.c_str());
    }
  };
} // namespace mrp_common

#endif