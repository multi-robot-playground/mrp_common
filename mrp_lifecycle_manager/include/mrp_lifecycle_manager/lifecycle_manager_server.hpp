#ifndef MULTI_ROBOT_LIFECYCLE_MANAGER__LIFECYLE_MANAGER_SRV_SERVER_HPP_
#define MULTI_ROBOT_LIFECYCLE_MANAGER__LIFECYLE_MANAGER_SRV_SERVER_HPP_

#include "mrp_common/lifecycle_node.hpp"
#include "mrp_common/service_server.hpp"
#include "mrp_common_msgs/srv/monitored_node.hpp"
#include "mrp_common_msgs/srv/monitored_node_array.hpp"

namespace mrp_lifecycle_manager
{
  class LifecycleManagerServer
  {
    public:
      LifecycleManagerServer(rclcpp_lifecycle::LifecycleNode::SharedPtr node);
      ~LifecycleManagerServer();

      bool changeMonitoredNodeState(const mrp_common::LifecycleNode::State desired_state);
      bool changeMonitoredNodesState(const std::vector<mrp_common::LifecycleNode::State> desired_states);

    protected:

      std::string change_node_state_srv_name_;
      std::string change_nodes_state_srv_name_;
      std::vector<std::string>
      std::shared_ptr<mrp_common::ServiceServer<mrp_common_msgs::srv::MonitoredNode>> change_node_state_server_;
      std::shared_ptr<mrp_common::ServiceServer<mrp_common_msgs::srv::MonitoredNodeArray>> change_nodes_state_server_;  
  };
}

#endif