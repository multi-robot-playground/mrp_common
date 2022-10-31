#include "mrp_common/lifecycle_node.hpp"

namespace mrp_common
{
  LifecycleNode::LifecycleNode(
      const std::string node_name,
      const std::string ns,
      const bool auto_start,
      const bool health_check,
      const std::chrono::milliseconds heartbeat_interval,
      const rclcpp::NodeOptions &options)
      : rclcpp_lifecycle::LifecycleNode(node_name, ns, options),
        auto_start_(auto_start),
        health_check_(health_check),
        heartbeat_interval_(heartbeat_interval)
  {
  }

  LifecycleNode::~LifecycleNode()
  {
  }

  void LifecycleNode::initialiseHeartbeat()
  {
    this->get_current_state();
    if (health_check_)
    {
      heartbeat_ptr_ = std::make_shared<Heartbeat>(
          heartbeat_interval_,
          get_node_topics_interface(),
          get_node_base_interface(),
          get_node_timers_interface(),
          get_node_clock_interface());
      heartbeat_ptr_->createHeartbeat();
    }
  }

  void LifecycleNode::startHeartbeat()
  { 
    return heartbeat_ptr_->startBeating();
  }

  void LifecycleNode::stopHeartbeat()
  {
    return heartbeat_ptr_->stopBeating();
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr LifecycleNode::sharedFromThis()
  {
    std::cout << this->shared_from_this() << std::endl;
    return  this->shared_from_this();
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  LifecycleNode::on_configure(const rclcpp_lifecycle::State &)
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  LifecycleNode::on_activate(const rclcpp_lifecycle::State &)
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  LifecycleNode::on_deactivate(const rclcpp_lifecycle::State &)
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  LifecycleNode::on_cleanup(const rclcpp_lifecycle::State &)
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  LifecycleNode::on_shutdown(const rclcpp_lifecycle::State & state)
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }
    
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  LifecycleNode::on_error(const rclcpp_lifecycle::State &)
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  LifecycleNode::Heartbeat::Heartbeat(
      std::chrono::milliseconds interval,
      rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topic_interface,
      rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
      rclcpp::node_interfaces::NodeTimersInterface::SharedPtr node_timer_interface,
      rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface)
      : interval_(interval),
        node_topic_interface_(node_topic_interface),
        node_base_interface_(node_base_interface),
        node_timer_interface_(node_timer_interface),
        node_clock_interface_(node_clock_interface)
  {
  
  }

  LifecycleNode::Heartbeat::~Heartbeat()
  {
  }

  void LifecycleNode::Heartbeat::createHeartbeat()
  {
    node_name_ = node_base_interface_->get_fully_qualified_name();
    if (!heartbeat_publisher_)
    {
      // The granted lease is essentially infite here, i.e., only reader/watchdog will notify
      // violations. XXX causes segfault for cyclone dds, hence pass explicit lease life > heartbeat.
      rclcpp::QoS qos_profile(1);
      // qos_profile
      //     .liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC) // The topic itself has to be HEALTHY
      //     .liveliness_lease_duration(interval_)                  //
      //     .deadline(interval_);                                  //
      // assert liveliness on the 'heartbeat' topic
      // Minimal publisher for publishing heartbeats
      heartbeat_publisher_ = rclcpp::create_publisher<mrp_common_msgs::msg::Heartbeat>(
          node_topic_interface_, node_name_ + "/heartbeat", qos_profile);
    }
    if (!heartbeat_timer_)
    {
      heartbeat_timer_ = rclcpp::create_wall_timer(
          interval_,
          std::bind(&LifecycleNode::Heartbeat::heartbeatCallback, this),
          nullptr,
          node_base_interface_.get(),
          node_timer_interface_.get());

      stopBeating();
    }
  }

  void LifecycleNode::Heartbeat::startBeating()
  {
    if (heartbeat_timer_->is_canceled())
    {
      heartbeat_timer_->reset();
    }
  }

  void LifecycleNode::Heartbeat::stopBeating()
  {
    // heartbeat_timer_->cancel();
  }

  void LifecycleNode::Heartbeat::heartbeatCallback()
  {
    auto message = mrp_common_msgs::msg::Heartbeat();
    rclcpp::Time now = node_clock_interface_->get_clock()->now();
    message.stamp = now;
    // RCLCPP_INFO(node_ptr_->get_logger(), "Publishing heartbeat, sent at [%f]", now.seconds());
    heartbeat_publisher_->publish(message);
  }
}
