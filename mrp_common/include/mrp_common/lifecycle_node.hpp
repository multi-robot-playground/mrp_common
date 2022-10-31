#ifndef MULTI_ROBOT_PLAYGROUND_COMMON__LIFECYCLE_NODE_HPP_
#define MULTI_ROBOT_PLAYGROUND_COMMON__LIFECYCLE_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "mrp_common_msgs/msg/heartbeat.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

namespace mrp_common
{
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  class LifecycleNode : public rclcpp_lifecycle::LifecycleNode
  {
  public:
    enum class Transition
    {
      CREATE = lifecycle_msgs::msg::Transition::TRANSITION_CREATE,
      CONFIGURE = lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE,
      CLEANUP = lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP,
      ACTIVATE = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE,
      DEACTIVATE = lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE,
      UNCONFIGURED_SHUTDOWN = lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN,
      INACTIVE_SHUTDOWN = lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN,
      ACTIVE_SHUTDOWN = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN,
      DESTROY = lifecycle_msgs::msg::Transition::TRANSITION_DESTROY,

      ON_CONFIGURE_SUCCESS = lifecycle_msgs::msg::Transition::TRANSITION_ON_CONFIGURE_SUCCESS,
      ON_CONFIGURE_FAILURE = lifecycle_msgs::msg::Transition::TRANSITION_ON_CONFIGURE_FAILURE,
      ON_CONFIGURE_ERROR = lifecycle_msgs::msg::Transition::TRANSITION_ON_CONFIGURE_ERROR,

      ON_CLEANUP_SUCCESS = lifecycle_msgs::msg::Transition::TRANSITION_ON_CLEANUP_SUCCESS,
      ON_CLEANUP_FAILURE = lifecycle_msgs::msg::Transition::TRANSITION_ON_CLEANUP_FAILURE,
      ON_CLEANUP_ERROR = lifecycle_msgs::msg::Transition::TRANSITION_ON_CLEANUP_ERROR,

      ON_ACTIVATE_SUCCESS = lifecycle_msgs::msg::Transition::TRANSITION_ON_ACTIVATE_SUCCESS,
      ON_ACTIVATE_FAILURE = lifecycle_msgs::msg::Transition::TRANSITION_ON_ACTIVATE_FAILURE,
      ON_ACTIVATE_ERROR = lifecycle_msgs::msg::Transition::TRANSITION_ON_ACTIVATE_ERROR,

      ON_DEACTIVATE_SUCCESS = lifecycle_msgs::msg::Transition::TRANSITION_ON_DEACTIVATE_SUCCESS,
      ON_DEACTIVATE_FAILURE = lifecycle_msgs::msg::Transition::TRANSITION_ON_DEACTIVATE_FAILURE,
      ON_DEACTIVATE_ERROR = lifecycle_msgs::msg::Transition::TRANSITION_ON_DEACTIVATE_ERROR,

      ON_SHUTDOWN_SUCCESS = lifecycle_msgs::msg::Transition::TRANSITION_ON_SHUTDOWN_SUCCESS,
      ON_SHUTDOWN_FAILURE = lifecycle_msgs::msg::Transition::TRANSITION_ON_SHUTDOWN_FAILURE,
      ON_SHUTDOWN_ERROR = lifecycle_msgs::msg::Transition::TRANSITION_ON_SHUTDOWN_ERROR,

      ON_ERROR_SUCCESS = lifecycle_msgs::msg::Transition::TRANSITION_ON_ERROR_SUCCESS,
      ON_ERROR_FAILURE = lifecycle_msgs::msg::Transition::TRANSITION_ON_ERROR_FAILURE,
      ON_ERROR_ERROR = lifecycle_msgs::msg::Transition::TRANSITION_ON_ERROR_ERROR,

      CALLBACK_SUCCESS = lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_SUCCESS,
      CALLBACK_FAILURE = lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_FAILURE,
      CALLBACK_ERROR = lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_ERROR
    };

    enum class State
    {
      PRIMARY_STATE_UNKNOWN = lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN,
      PRIMARY_STATE_UNCONFIGURED = lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
      PRIMARY_STATE_INACTIVE = lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      PRIMARY_STATE_ACTIVE = lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      PRIMARY_STATE_FINALIZED = lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED,

      TRANSITION_STATE_CONFIGURING = lifecycle_msgs::msg::State::TRANSITION_STATE_CONFIGURING,
      TRANSITION_STATE_CLEANINGUP = lifecycle_msgs::msg::State::TRANSITION_STATE_CLEANINGUP,
      TRANSITION_STATE_SHUTTINGDOWN = lifecycle_msgs::msg::State::TRANSITION_STATE_SHUTTINGDOWN,
      TRANSITION_STATE_ACTIVATING = lifecycle_msgs::msg::State::TRANSITION_STATE_ACTIVATING,
      TRANSITION_STATE_DEACTIVATING = lifecycle_msgs::msg::State::TRANSITION_STATE_DEACTIVATING,
      TRANSITION_STATE_ERRORPROCESSING = lifecycle_msgs::msg::State::TRANSITION_STATE_ERRORPROCESSING,
    };
    
    LifecycleNode(const std::string node_name,
                  const std::string ns,
                  const bool auto_start,
                  const bool health_check,
                  const std::chrono::milliseconds heartbeat_interval,
                  const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    virtual ~LifecycleNode();

    void initialiseHeartbeat();
    void startHeartbeat();
    void stopHeartbeat();

    rclcpp_lifecycle::LifecycleNode::SharedPtr sharedFromThis();

    // We need these here or else inheritance will not work properly
    CallbackReturn on_configure(const rclcpp_lifecycle::State &state);
    CallbackReturn on_activate(const rclcpp_lifecycle::State &state);
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state);
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state);
    CallbackReturn on_error(const rclcpp_lifecycle::State &state);
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state);

  protected:
    class Heartbeat
    {
    public:
      Heartbeat(std::chrono::milliseconds interval,
                rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topic_interface,
                rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
                rclcpp::node_interfaces::NodeTimersInterface::SharedPtr node_timer_interface,
                rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface);
      ~Heartbeat();

      void createHeartbeat();
      void startBeating();
      void stopBeating();

    protected:
      rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topic_interface_;
      rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface_;
      rclcpp::node_interfaces::NodeTimersInterface::SharedPtr node_timer_interface_;
      rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface_;
      // Heartbeats
      std::chrono::milliseconds interval_;
      rclcpp::Publisher<mrp_common_msgs::msg::Heartbeat>::SharedPtr heartbeat_publisher_{nullptr};
      rclcpp::TimerBase::SharedPtr heartbeat_timer_{nullptr};

      std::string node_name_;

      void heartbeatCallback();
    };

    bool health_check_;
    bool auto_start_;

    std::shared_ptr<Heartbeat> heartbeat_ptr_;
    std::chrono::milliseconds heartbeat_interval_;
  };
}

#endif