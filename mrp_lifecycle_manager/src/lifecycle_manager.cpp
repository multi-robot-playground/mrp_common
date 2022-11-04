#include "mrp_lifecycle_manager/lifecyle_manager.hpp"

namespace mrp_lifecycle_manager
{
  // Aliases for ease of use
  using LifecycleTransition = mrp_common::LifecycleNode::Transition;
  using LifecycleState = mrp_common::LifecycleNode::State;

  using namespace std::chrono_literals;

  LifecycleManager::LifecycleManager(rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_ptr,
                                     const rclcpp::NodeOptions &options,
                                     std::chrono::milliseconds heartbeat_timeout)
      : rclcpp_lifecycle::LifecycleNode("lifecycle_manager", options),
        executor_ptr_(executor_ptr),
        heartbeat_timeout_(heartbeat_timeout)
  {
    // Main normal state
    transition_state_map_[LifecycleTransition::CREATE] = LifecycleState::PRIMARY_STATE_UNCONFIGURED;
    transition_state_map_[LifecycleTransition::CONFIGURE] = LifecycleState::PRIMARY_STATE_INACTIVE;
    transition_state_map_[LifecycleTransition::ACTIVATE] = LifecycleState::PRIMARY_STATE_ACTIVE;
    transition_state_map_[LifecycleTransition::DEACTIVATE] = LifecycleState::PRIMARY_STATE_INACTIVE;
    transition_state_map_[LifecycleTransition::ACTIVE_SHUTDOWN] = LifecycleState::PRIMARY_STATE_UNKNOWN;

    heartbeat_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // executor_ptr_->add_node(get_node_base_interface());
    executor_ptr_->add_node(get_node_base_interface());

    declare_parameter<std::vector<std::string>>("node_names", std::vector<std::string>());
    declare_parameter<int>("monitor_rate", 1000);
    declare_parameter<bool>("autostart", false);
    declare_parameter<std::vector<double>>("heartbeat_interval", std::vector<double>());
    declare_parameter<double>("bond_respawn_max_duration", 10.0);
    declare_parameter<bool>("attempt_respawn_reconnection", true);
  }

  LifecycleManager::~LifecycleManager()
  {
  }

  void LifecycleManager::spin()
  {
    execution_future_ = std::async(std::launch::async, [this]()
                                   { executor_ptr_->spin(); });
  }

  bool LifecycleManager::registerLifecycleNode(const std::string &node_name,
                                               const std::chrono::milliseconds &heartbeat_interval)
  {
    if (monitored_node_map_.find(node_name) == monitored_node_map_.end() &&
        heartbeat_timeout_.count() > 0.0)
    {
      mrp_common::Log::basicInfo(get_node_logging_interface(),
                                 "Registering " + node_name);
      MonitoredNode monitored_node;
      monitored_node.heartbeat_ptr_ = std::make_shared<LifecycleManager::HealthMonitor>(
          node_name,
          heartbeat_timeout_,
          heartbeat_interval,
          false,
          callback_group_,
          get_node_topics_interface(),
          get_node_base_interface(),
          get_node_timers_interface(),
          get_node_clock_interface());

      // Initialise heartbeat monitoring
      monitored_node.heartbeat_ptr_->initialiseHealthMonitor();

      // Create lifecycle_manager_client for requesting state changes
      monitored_node.lifecyle_manager_client_ = std::make_shared<LifecycleManagerClient>(
          shared_from_this(),
          node_name);

      // Add to the monitored map
      monitored_node_map_[node_name] = monitored_node;

      return true;
    }
    return false;
  }

  bool LifecycleManager::removeLifecycleNode(const std::string &node_name)
  {
    monitored_node_map_.erase(node_name);
  }

  bool LifecycleManager::registerLifecycleNodes(
      const std::vector<std::string> &monitored_node_names,
      const std::vector<std::chrono::milliseconds> &heartbeat_intervals)
  {
    for (unsigned int idx = 0; idx < monitored_node_names.size(); idx++)
    {
      // Manually register each lifecycle node
      if (!registerLifecycleNode(monitored_node_names.at(idx), heartbeat_intervals.at(idx)))
      {
        return false;
      }
    }
    return true;
  }

  bool LifecycleManager::removeLifecycleNodes(const std::vector<std::string> &monitored_node_names)
  {
    for (const std::string node_name : monitored_node_names)
    {
      removeLifecycleNode(node_name);
    }
  }

  bool LifecycleManager::startNodeHealthMonitor(const std::string &node_name)
  {
    bool unhealthy_node_exist = false;
    if (!monitored_node_map_[node_name].heartbeat_ptr_->initialiseHealthMonitor())
    {
      // Log error message
      unhealthy_node_exist = true;
      // Move on to the next node and flag this node
    }
    return unhealthy_node_exist;
  }

  bool LifecycleManager::startNodesHealthMonitor(const std::vector<std::string> &monitored_node_names)
  {
    bool all_node_healthy = true;
    for (const std::string &node_name : monitored_node_names)
    {
      // Manually register each lifecycle node
      all_node_healthy = startNodeHealthMonitor(node_name);
    }
    return all_node_healthy;
  }

  LifecycleManager::TransitionRequestStatus LifecycleManager::transitionNode(const std::string &node_name,
                                                                             LifecycleTransition transition,
                                                                             std::chrono::milliseconds timeout)
  {
    if (!monitored_node_map_[node_name].lifecyle_manager_client_->requestTransition(transition, timeout))
    {
      return TransitionRequestStatus::CANNOT_CHANGE_STATE;
    }
    // Compare with the desired target state
    // Since the timeout has be incorporate in the service call, let's just check for the state at
    // this point
    if (getNodeState(node_name, timeout) != transition_state_map_[transition])
    {
      return TransitionRequestStatus::WRONG_END_STATE;
    }
    return TransitionRequestStatus::NO_ERROR;
  }

  bool LifecycleManager::transitionNodes(const std::vector<std::string> &monitored_node_names,
                                         LifecycleTransition transition,
                                         std::chrono::milliseconds timeout)
  {
    for (const std::string &node_name : monitored_node_names)
    {
      if (transitionNode(node_name, transition, timeout) != TransitionRequestStatus::NO_ERROR)
      {
        return false;
      }
    }

    return true;
  }

  mrp_common::LifecycleNode::State LifecycleManager::getNodeState(const std::string &node_name,
                                                                  std::chrono::milliseconds timeout)
  {
    return monitored_node_map_[node_name].lifecyle_manager_client_->getNodeState(timeout);
  }

  // LIFECYCLE NODE STATE TRANSITION CALLBACK
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  LifecycleManager::on_configure(const rclcpp_lifecycle::State &state)
  {
    // Request params from local params server
    mrp_common::Log::basicInfo(get_node_logging_interface(),
                               "Configuring lifecycle manager");

    // Get all parameters
    loadParameters();

    // Get necessary parameters
    bool auto_start = false;
    get_parameter("autostart", auto_start);
    std::vector<double> heartbeat_interval;
    get_parameter("heartbeat_interval", heartbeat_interval);

    // Convert raw duration in double to chrono duration
    std::vector<std::chrono::milliseconds> converted_heartbeat;
    for (const auto heartbeat : heartbeat_interval)
    {
      converted_heartbeat.push_back(std::chrono::duration<int64_t, std::milli>((int64_t)heartbeat));
    }

    if (converted_heartbeat.size() != monitored_node_names_.size())
    {
      mrp_common::Log::basicError(get_node_logging_interface(),
                                  "Unable to register monitor timer due to: \
                                  Mismatch number of nodes and heartbeat values. \
                                  Exitting...");
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
    // Register lifecycle nodes
    if (!registerLifecycleNodes(monitored_node_names_, converted_heartbeat))
    {
      mrp_common::Log::basicError(get_node_logging_interface(),
                                  "Unable to register monitor timer");
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

    // Create monitored_node_array service for changing states of monitored node dynamically
    change_nodes_state_server_ = std::make_shared<mrp_common::ServiceServer<mrp_common_msgs::srv::MonitoredNodeArray>>(
        shared_from_this(),
        std::string(get_name()) + "/change_monitored_nodes_state",
        std::bind(&LifecycleManager::changeNodesStateCallback, this, std::placeholders::_1, std::placeholders::_2),
        false, rcl_service_get_default_options());

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  LifecycleManager::on_activate(const rclcpp_lifecycle::State &state)
  {
    // Hard code 500ms for state transition for now
    std::chrono::milliseconds timeout(500);
    // Move all nodes to configuring state state
    if (!transitionNodes(monitored_node_names_, LifecycleTransition::CONFIGURE, timeout))
    {
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

    // Move all nodes to activation state
    if (!transitionNodes(monitored_node_names_, LifecycleTransition::ACTIVATE, timeout))
    {
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

    // // Create wall timer for health monitoring
    // if (!createMonitorTimer())
    // {
    //   // SHould log here
    //   return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    // }
    system_active_ = true;
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  LifecycleManager::on_deactivate(const rclcpp_lifecycle::State &state)
  {
    system_active_ = false;
    // Hard code 500ms for state transition for now
    std::chrono::milliseconds timeout(500);
    // Move all nodes to deactivating state
    if (!transitionNodes(monitored_node_names_, LifecycleTransition::DEACTIVATE, timeout))
    {
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  LifecycleManager::on_cleanup(const rclcpp_lifecycle::State &state)
  {
    system_active_ = false;
    // Hard code 500ms for state transition for now
    std::chrono::milliseconds timeout(500);
    // Move all nodes to deactivating state
    if (!transitionNodes(monitored_node_names_, LifecycleTransition::DEACTIVATE, timeout))
    {
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

    if (!transitionNodes(monitored_node_names_, LifecycleTransition::CLEANUP, timeout))
    {
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  LifecycleManager::on_error(const rclcpp_lifecycle::State &state)
  {
  }
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  LifecycleManager::on_shutdown(const rclcpp_lifecycle::State &state)
  {
  }
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  LifecycleManager::on_exit(const rclcpp_lifecycle::State &state)
  {
  }

  void LifecycleManager::monitorNodes()
  {
    for (auto const &monitored_node : monitored_node_map_)
    {
      if (!rclcpp::ok())
      {
        return;
      }
      // Check heartbeat if healthy or not
      if (monitored_node.second.heartbeat_ptr_->getStatus() != HealthMonitor::NodeHeartbeat::HEALTHY)
      {
        // It is not healthy at all what should we do
        mrp_common::Log::basicError(get_node_logging_interface(),
                                    std::string("Have not received a heartbeat from " + monitored_node.first + "."));
      }
    }
  }

  void LifecycleManager::loadParameters()
  {
    monitored_node_names_ = get_parameter("node_names").as_string_array();
  }

  bool LifecycleManager::createMonitorTimer()
  {
    // Destroy the old timer
    destroyMonitorTimer();
    // Start executor beyond this point

    // Let's monitor once every 1s
    monitor_timer_ = create_wall_timer(
        std::chrono::milliseconds(1000),
        [this]() -> void
        {
          // Start monitoring
          monitorNodes();
        },
        callback_group_);
    return true;
  }

  bool LifecycleManager::destroyMonitorTimer()
  {
    if (monitor_timer_)
    {
      // mrp_common::Log::basicInfo("Terminating timer...");
      monitor_timer_->cancel();
      monitor_timer_.reset();
    }
    return true;
  }

  void LifecycleManager::changeNodesStateCallback(std::shared_ptr<mrp_common_msgs::srv::MonitoredNodeArray::Request> &request,
                                                  std::shared_ptr<mrp_common_msgs::srv::MonitoredNodeArray::Response> &response)
  {
    if (!system_active_)
    {
      mrp_common::Log::basicError(
        get_node_logging_interface(),
        "Lifecycle Manager is inactive. Rejecting this request.");
        response->success = false;
        return;
    }

    bool success = true;
    for (auto node : request->nodes)
    {
      switch (node.command)
      {
      case mrp_common_msgs::srv::MonitoredNodeArray::Request::CONFIGURE:
        if (transitionNode(node.name, mrp_common::LifecycleNode::Transition::CONFIGURE,
                           std::chrono::milliseconds(1000)) != TransitionRequestStatus::NO_ERROR)
        {
          success = false;
        }
        break;

      case mrp_common_msgs::srv::MonitoredNodeArray::Request::ACTIVATE:
        if (transitionNode(node.name, mrp_common::LifecycleNode::Transition::ACTIVATE,
                           std::chrono::milliseconds(1000)) != TransitionRequestStatus::NO_ERROR)
        {
          success = false;
        }
        break;

      case mrp_common_msgs::srv::MonitoredNodeArray::Request::DEACTIVATE:
        if (transitionNode(node.name, mrp_common::LifecycleNode::Transition::DEACTIVATE,
                           std::chrono::milliseconds(1000)) != TransitionRequestStatus::NO_ERROR)
        {
          success = false;
        }
        break;
      case mrp_common_msgs::srv::MonitoredNodeArray::Request::CLEANUP:
        if (transitionNode(node.name, mrp_common::LifecycleNode::Transition::CLEANUP,
                           std::chrono::milliseconds(1000)) != TransitionRequestStatus::NO_ERROR)
        {
          success = false;
        }
        break;
      case mrp_common_msgs::srv::MonitoredNodeArray::Request::SHUTDOWN:
        if (transitionNode(node.name, mrp_common::LifecycleNode::Transition::INACTIVE_SHUTDOWN,
                           std::chrono::milliseconds(1000)) != TransitionRequestStatus::NO_ERROR)
        {
          success = false;
        }
        break;
      }
    }
    response->success = success;
  }

  //=================================================//
  //                                                 //
  //=================================================//

  LifecycleManager::HealthMonitor::HealthMonitor(
      const std::string node_name,
      std::chrono::milliseconds heartbeat_timeout,
      std::chrono::milliseconds heartbeat_interval,
      bool isolated_spin,
      rclcpp::CallbackGroup::SharedPtr callback_group,
      rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topic_interface,
      rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
      rclcpp::node_interfaces::NodeTimersInterface::SharedPtr node_timer_interface,
      rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface)
      : node_name_(node_name),
        heartbeat_timeout_(heartbeat_timeout),
        heartbeat_interval_(heartbeat_interval),
        isolated_spin_(isolated_spin),
        callback_group_(callback_group),
        node_topic_interface_(node_topic_interface),
        node_base_interface_(node_base_interface),
        node_timer_interface_(node_timer_interface),
        node_clock_interface_(node_clock_interface),
        qos_profile_(10)
  {
    heartbeat_interval_ms_ =
        std::chrono::duration_cast<std::chrono::milliseconds>(heartbeat_interval_).count();
    if (isolated_spin_)
    {
      callback_group_ = node_base_interface_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    }
    status_ = NodeHeartbeat::UNHEALTHY;
  }

  LifecycleManager::HealthMonitor::~HealthMonitor()
  {
  }

  bool LifecycleManager::HealthMonitor::initialiseHealthMonitor()
  {
    qos_profile_
        .liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC)
        .liveliness_lease_duration(heartbeat_interval_);
    heartbeat_sub_options_.event_callbacks.liveliness_callback =
        [this](rclcpp::QOSLivelinessChangedInfo &event) -> void
    {
      printf("Reader Liveliness changed event: \n");
      printf("  alive_count: %d\n", event.alive_count);
      printf("  not_alive_count: %d\n", event.not_alive_count);
      printf("  alive_count_change: %d\n", event.alive_count_change);
      printf("  not_alive_count_change: %d\n", event.not_alive_count_change);
      if (event.alive_count == 0)
      {
        status_ = NodeHeartbeat::UNKNOWN;
      }
    };

    rclcpp::QoS profile(1);
    std::string topic_name = node_name_ + "/heartbeat";
    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group_;

    heartbeat_sub_ = rclcpp::create_subscription<mrp_common_msgs::msg::Heartbeat>(
        node_topic_interface_,
        topic_name,
        profile,
        std::bind(&LifecycleManager::HealthMonitor::healthCallback, this, std::placeholders::_1),
        options);
    last_monitored_time_ = node_clock_interface_->get_clock()->now().nanoseconds();
  }

  void LifecycleManager::HealthMonitor::startMonitoring()
  {
    // Spin a default executor if isolate spin
    if (!isolated_spin_)
    {
      return;
    }

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_base_interface_, false);
    execution_future_ = std::async(std::launch::async, [this]()
                                   { executor_->spin(); });
  }

  void LifecycleManager::HealthMonitor::healthCallback(const typename mrp_common_msgs::msg::Heartbeat::SharedPtr msg)
  {
    std::cout << "Heartbeat received" << std::endl;
    if ((last_monitored_time_ - msg->stamp.nanosec) / 1e6 < heartbeat_interval_ms_)
    {
      status_ = NodeHeartbeat::HEALTHY;
      last_monitored_time_ = node_clock_interface_->get_clock()->now().nanoseconds();
      return;
    }
    status_ = NodeHeartbeat::UNHEALTHY;
    last_monitored_time_ = node_clock_interface_->get_clock()->now().nanoseconds();
    return;
  }

  LifecycleManager::HealthMonitor::NodeHeartbeat LifecycleManager::HealthMonitor::getStatus()
  {
    return status_;
  }
} // namespace mrp_lifecycle_manager
