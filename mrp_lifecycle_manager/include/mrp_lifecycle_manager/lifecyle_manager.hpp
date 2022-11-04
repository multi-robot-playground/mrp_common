#ifndef MULTI_ROBOT_LIFECYCLE_MANAGER__MULTI_ROBOT_LIFECYLE_MANAGER_HPP_
#define MULTI_ROBOT_LIFECYCLE_MANAGER__MULTI_ROBOT_LIFECYLE_MANAGER_HPP_

#include <chrono>
#include <thread>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "mrp_common/lifecycle_node.hpp"
#include "mrp_common_msgs/msg/heartbeat.hpp"
#include "mrp_common/logging.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "mrp_lifecycle_manager/lifecycle_manager_client.hpp"

#include "mrp_common/service_server.hpp"
#include "mrp_common_msgs/srv/monitored_node_array.hpp"

namespace mrp_lifecycle_manager
{
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  class LifecycleManager : public rclcpp_lifecycle::LifecycleNode
  {

  public:
    enum class TransitionRequestStatus
    {
      NO_ERROR = 0,
      CANNOT_CHANGE_STATE = 1,
      WRONG_END_STATE = 2
    };

    explicit LifecycleManager(rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_ptr,
                              const rclcpp::NodeOptions &options,
                              std::chrono::milliseconds heartbeat_timeout);
    virtual ~LifecycleManager();

    void spin();

    bool registerLifecycleNode(const std::string &node_name,
                               const std::chrono::milliseconds &heartbeat_interval);
    bool removeLifecycleNode(const std::string &node_name);

    bool registerLifecycleNodes(
        const std::vector<std::string> &monitored_node_names,
        const std::vector<std::chrono::milliseconds> &heartbeat_intervals);

    bool removeLifecycleNodes(const std::vector<std::string> &monitored_node_names);

    bool startNodeHealthMonitor(const std::string &node_name);
    bool startNodesHealthMonitor(const std::vector<std::string> &monitored_node_names);

    TransitionRequestStatus transitionNode(const std::string &node_name,
                                           mrp_common::LifecycleNode::Transition transition,
                                           std::chrono::milliseconds timeout);
    bool transitionNodes(const std::vector<std::string> &monitored_node_names,
                         mrp_common::LifecycleNode::Transition transition,
                         std::chrono::milliseconds timeout);

    mrp_common::LifecycleNode::State getNodeState(const std::string &node_name,
                                                  std::chrono::milliseconds timeout);

    CallbackReturn on_configure(const rclcpp_lifecycle::State &state);
    CallbackReturn on_activate(const rclcpp_lifecycle::State &state);
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state);
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state);
    CallbackReturn on_error(const rclcpp_lifecycle::State &state);
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state);
    CallbackReturn on_exit(const rclcpp_lifecycle::State &state);

    void monitorNodes();
    void loadParameters();

    // Heartbeat related functions
    bool createMonitorTimer();
    bool destroyMonitorTimer();

  protected:
    class HealthMonitor
    {
    public:
      enum class NodeHeartbeat
      {
        HEALTHY = 0,
        UNHEALTHY = 1,
        UNKNOWN = 2
      };

      HealthMonitor(
          const std::string node_name,
          std::chrono::milliseconds heartbeat_timeout,
          std::chrono::milliseconds heartbeat_interval,
          bool isolated_spin,
          rclcpp::CallbackGroup::SharedPtr callback_group,
          rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topic_interface,
          rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
          rclcpp::node_interfaces::NodeTimersInterface::SharedPtr node_timer_interface,
          rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface);
      ~HealthMonitor();

      NodeHeartbeat getStatus();
      bool initialiseHealthMonitor();
      void startMonitoring();
      void stopMonitoring();

      int test = 0;

    protected:
      /// The lease duration granted to the remote (heartbeat) publisher
      std::chrono::milliseconds heartbeat_timeout_;
      std::chrono::milliseconds heartbeat_interval_;
      rclcpp::Subscription<mrp_common_msgs::msg::Heartbeat>::SharedPtr heartbeat_sub_;
      rclcpp::QoS qos_profile_;
      rclcpp::SubscriptionOptions heartbeat_sub_options_;

      double last_monitored_time_;
      double heartbeat_interval_ms_;
      mutable std::recursive_mutex status_mutex_;
      std::future<void> execution_future_;
      NodeHeartbeat status_;
      bool isolated_spin_;

      std::string node_name_;
      rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topic_interface_;
      rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface_;
      rclcpp::node_interfaces::NodeTimersInterface::SharedPtr node_timer_interface_;
      rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface_;

      void healthCallback(const typename mrp_common_msgs::msg::Heartbeat::SharedPtr msg);

      rclcpp::CallbackGroup::SharedPtr callback_group_;
      rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
    };

    struct MonitoredNode
    {
      std::shared_ptr<HealthMonitor> heartbeat_ptr_;
      std::shared_ptr<LifecycleManagerClient> lifecyle_manager_client_;
    };

    bool system_active_{false};
    std::future<void> execution_future_;

    std::chrono::milliseconds heartbeat_timeout_;
    std::vector<std::string> monitored_node_names_;
    std::map<std::string, MonitoredNode> monitored_node_map_;

    rclcpp::TimerBase::SharedPtr monitor_timer_;
    rclcpp::CallbackGroup::SharedPtr heartbeat_callback_group_{nullptr};
    rclcpp::CallbackGroup::SharedPtr callback_group_{nullptr};

    rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_ptr_;

    std::map<mrp_common::LifecycleNode::Transition, mrp_common::LifecycleNode::State> transition_state_map_;
    std::shared_ptr<mrp_common::ServiceServer<mrp_common_msgs::srv::MonitoredNodeArray>> change_nodes_state_server_;

    void changeNodesStateCallback(std::shared_ptr<mrp_common_msgs::srv::MonitoredNodeArray::Request> &request,
                                  std::shared_ptr<mrp_common_msgs::srv::MonitoredNodeArray::Response> &response);
  };
} // namespace mrp_lifecycle_manager

#endif // MULTI_ROBOT_LIFECYCLE_MANAGER__MULTI_ROBOT_LIFECYLE_MANAGER_HPP_