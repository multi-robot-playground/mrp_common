#include "mrp_lifecycle_manager/lifecycle_manager_client.hpp"

namespace mrp_lifecycle_manager
{
  LifecycleManagerClient::LifecycleManagerClient(rclcpp_lifecycle::LifecycleNode::SharedPtr node,
                                                 std::string managed_node_name)
      : node_(node)
  {
    change_state_service_name_ = managed_node_name + "/change_state";
    change_state_client_ = std::make_shared<mrp_common::ServiceClient<lifecycle_msgs::srv::ChangeState>>(
        node_, false, change_state_service_name_, rcl_service_get_default_options());

    get_state_service_name_ = managed_node_name + "/get_state";
    get_state_client_ = std::make_shared<mrp_common::ServiceClient<lifecycle_msgs::srv::GetState>>(
        node_, false, get_state_service_name_, rcl_service_get_default_options());
  }

  LifecycleManagerClient::~LifecycleManagerClient()
  {
  }

  bool LifecycleManagerClient::requestTransition(mrp_common::LifecycleNode::Transition transition,
                                                 const std::chrono::milliseconds timeout)
  {
    lifecycle_msgs::srv::ChangeState::Request::SharedPtr request =
        std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    lifecycle_msgs::srv::ChangeState::Response::SharedPtr response =
        std::make_shared<lifecycle_msgs::srv::ChangeState::Response>();
    request->transition.id = (uint8_t)transition;

    // This should block
    try
    {
      return change_state_client_->requestAndWaitForResponse(request, response);
    }
    catch (std::exception &ex)
    {
      std::cout << ex.what() << std::endl;
    }
    return false;
  }

  bool LifecycleManagerClient::requestConfigure(const std::chrono::milliseconds timeout)
  {
    return requestTransition(mrp_common::LifecycleNode::Transition::CONFIGURE, timeout);
  }

  bool LifecycleManagerClient::requestActivate(const std::chrono::milliseconds timeout)
  {
    return requestTransition(mrp_common::LifecycleNode::Transition::ACTIVATE, timeout);
  }

  bool LifecycleManagerClient::requestDeactivate(const std::chrono::milliseconds timeout)
  {
    return requestTransition(mrp_common::LifecycleNode::Transition::DEACTIVATE, timeout);
  }

  bool LifecycleManagerClient::requestUnconfiguredShutdown(const std::chrono::milliseconds timeout)
  {
    return requestTransition(mrp_common::LifecycleNode::Transition::UNCONFIGURED_SHUTDOWN, timeout);
  }

  bool LifecycleManagerClient::requestInactiveShutdown(const std::chrono::milliseconds timeout)
  {
    return requestTransition(mrp_common::LifecycleNode::Transition::INACTIVE_SHUTDOWN, timeout);
  }

  bool LifecycleManagerClient::requestActiveShutdown(const std::chrono::milliseconds timeout)
  {
    return requestTransition(mrp_common::LifecycleNode::Transition::ACTIVE_SHUTDOWN, timeout);
  }

  mrp_common::LifecycleNode::State
  LifecycleManagerClient::getNodeState(const std::chrono::milliseconds timeout)
  {
    lifecycle_msgs::srv::GetState::Request::SharedPtr request =
        std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    lifecycle_msgs::srv::GetState::Response::SharedPtr response =
        std::make_shared<lifecycle_msgs::srv::GetState::Response>();

    get_state_client_->requestAndWaitForResponse(request, response, timeout);
    return static_cast<mrp_common::LifecycleNode::State>(response->current_state.id);
  }
}