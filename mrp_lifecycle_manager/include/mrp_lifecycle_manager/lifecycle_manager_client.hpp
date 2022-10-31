#ifndef MULTI_ROBOT_LIFECYCLE_MANAGER__LIFECYLE_MANAGER_CLIENT_HPP_
#define MULTI_ROBOT_LIFECYCLE_MANAGER__LIFECYLE_MANAGER_CLIENT_HPP_

#include <chrono>

#include "mrp_common/service_client.hpp"
#include "mrp_common/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

namespace mrp_lifecycle_manager
{
  class LifecycleManagerClient
  {
  public:
    LifecycleManagerClient(rclcpp_lifecycle::LifecycleNode::SharedPtr node,
                           std::string managed_node_name);
    ~LifecycleManagerClient();

    bool requestTransition(mrp_common::LifecycleNode::Transition transition,
                           const std::chrono::milliseconds timeout);
    
    bool requestConfigure(const std::chrono::milliseconds timeout);
    bool requestCleanup(const std::chrono::milliseconds timeout);
    bool requestActivate(const std::chrono::milliseconds timeout);
    bool requestDeactivate(const std::chrono::milliseconds timeout);
    bool requestUnconfiguredShutdown(const std::chrono::milliseconds timeout);
    bool requestInactiveShutdown(const std::chrono::milliseconds timeout);
    bool requestActiveShutdown(const std::chrono::milliseconds timeout);

    mrp_common::LifecycleNode::State getNodeState(const std::chrono::milliseconds timeout);

  protected:
    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
    std::string change_state_service_name_;
    std::string get_state_service_name_;

    std::shared_ptr<mrp_common::ServiceClient<lifecycle_msgs::srv::ChangeState>> change_state_client_;
    std::shared_ptr<mrp_common::ServiceClient<lifecycle_msgs::srv::GetState>> get_state_client_;
  };

}

#endif