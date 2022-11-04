#ifndef MULTI_ROBOT_PLAYGROUND_COMMON__SERVICE_CLIENT_HPP_
#define MULTI_ROBOT_PLAYGROUND_COMMON__SERVICE_CLIENT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "logging.hpp"
#include <iostream>

namespace mrp_common
{
  template <typename ServiceType>
  class ServiceClient
  {
  public:
    template <typename NodeType>
    explicit ServiceClient(
        NodeType node,
        bool isolated_spin,
        const std::string &service_name,
        const rcl_service_options_t &options)
        : node_base_interface_(node->get_node_base_interface()),
          node_graph_interface_(node->get_node_graph_interface()),
          node_services_interface_(node->get_node_services_interface()),
          node_logging_interface_(node->get_node_logging_interface()),
          service_name_(service_name),
          isolated_spin_(isolated_spin)
    {
      callback_group_ = node->create_callback_group(
          rclcpp::CallbackGroupType::MutuallyExclusive);
      if(isolated_spin_)
      {
        executor_.add_node(node->get_node_base_interface());
      }
      service_client_ = rclcpp::create_client<ServiceType>(
          node_base_interface_,
          node_graph_interface_,
          node_services_interface_,
          service_name_,
          rmw_qos_profile_default,
          callback_group_);
    }

    virtual ~ServiceClient() {}

    bool requestAndWaitForResponse(typename ServiceType::Request::SharedPtr &request,
                                   typename ServiceType::Response::SharedPtr &response,
                                   const std::chrono::milliseconds timeout = std::chrono::milliseconds(1000))
    {
      // Wait for service server to response
      // Question: Should this function be a blocking function ?
      // We should handle server timeout cases
      Log::basicInfo(node_logging_interface_, "Checking if server is ready.");
      while (!service_client_->wait_for_service(std::chrono::seconds(1)))
      {
        if (!rclcpp::ok())
        {
          Log::basicError(node_logging_interface_, "Client interrupted while waiting for service to appear.");
          return false;
        }
        Log::basicInfo(node_logging_interface_, "Waiting for service to appear...");
      }
      response_ready_ = false;

      // If isolated spin then we spin up a separate executor and start sending request
      if (isolated_spin_)
      {
        auto future_result = service_client_->async_send_request(request);
        if (executor_.spin_until_future_complete(future_result, timeout) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
          throw std::runtime_error(service_name_ + " service client: async_send_request failed");
        }

        response = future_result.get();
        return response.get();
      }

      auto result_future = service_client_->async_send_request(
          request, std::bind(&ServiceClient::responseCallback, this,
                             std::placeholders::_1));

      Log::basicInfo(node_logging_interface_, "Waiting for response");
      while (result_future.wait_for(timeout) != std::future_status::ready);
      Log::basicInfo(node_logging_interface_, "Response ready.");
      response = result_future.get();
      return true;
    }

  protected:
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface_;
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_interface_;
    rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services_interface_;
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface_;

    std::string service_name_;
    bool isolated_spin_;
    std::atomic_bool response_ready_;

    typename rclcpp::Client<ServiceType>::SharedPtr service_client_;
    typename ServiceType::Response::SharedPtr response_;

    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    std::chrono::milliseconds timeout_;

    void responseCallback(typename rclcpp::Client<ServiceType>::SharedFuture shared_future)
    {
      response_ = shared_future.get();
    }
  };
}
#endif