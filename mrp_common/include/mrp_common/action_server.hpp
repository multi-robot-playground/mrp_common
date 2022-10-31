#ifndef MULTI_ROBOT_PLAYGROUND_COMMON__ACTION_SERVER_HPP_
#define MULTI_ROBOT_PLAYGROUND_COMMON__ACTION_SERVER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <future>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "logging.hpp"

namespace mrp_common
{
  template <typename ActionType>
  class ActionServer
  {
  public:
    // Callback function to complete main work. This should itself deal with its
    // own exceptions, but if for some reason one is thrown, it will be caught
    // in SimpleActionServer and terminate the action itself.
    typedef std::function<void()> ExecuteCallback;

    // Callback function to notify the user that an exception was thrown that
    // the simple action server caught (or another failure) and the action was
    // terminated. To avoid using, catch exceptions in your application such that
    // the SimpleActionServer will never need to terminate based on failed action
    // ExecuteCallback.
    typedef std::function<void()> CompletionCallback;

    template <typename NodeSharedPtrType>
    explicit ActionServer(
        NodeSharedPtrType &node,
        const std::string &action_name,
        ExecuteCallback execute_callback,
        CompletionCallback completion_callback,
        std::chrono::milliseconds server_timeout,
        bool spin_thread,
        const rcl_action_server_options_t &options)
        : node_base_interface_(node->get_node_base_interface()),
          node_clock_interface_(node->get_node_clock_interface()),
          node_logging_interface_(node->get_node_logging_interface()),
          node_waitables_interface_(node->get_node_waitables_interface()),
          action_name_(action_name),
          execute_callback_(execute_callback),
          completion_callback_(completion_callback),
          spin_thread_(spin_thread),
          server_active_(false),
          stop_execution_(false),
          preempt_requested_(false)
    {
      using namespace std::placeholders;
      if (spin_thread_)
      {
        callback_group_ = node_base_interface_->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        callback_group_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        callback_group_executor_->add_node(node_base_interface_);
      }
      action_server_ = rclcpp_action::create_server<ActionType>(
          node_base_interface_,
          node_clock_interface_,
          node_logging_interface_,
          node_waitables_interface_,
          action_name_,
          std::bind(&ActionServer::handleGoal, this, _1, _2),
          std::bind(&ActionServer::handleCancel, this, _1),
          std::bind(&ActionServer::handleAccepted, this, _1),
          options,
          callback_group_);
    }

    virtual ~ActionServer()
    {
    }

    rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID &uuid,
                                           std::shared_ptr<const typename ActionType::Goal> goal)
    {
      std::lock_guard<std::recursive_mutex> lck_guard(update_mutex_);
      if (!server_active_)
      {
        Log::basicWarn(
            node_logging_interface_,
            "Received goal but server is inactive, so reject this goal.");
        return rclcpp_action::GoalResponse::REJECT;
      }

      Log::basicInfo(
          node_logging_interface_,
          "Received request for goal acceptance.");
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionType>> handle)
    {
      std::lock_guard<std::recursive_mutex> lck_guard(update_mutex_);
      if (!handle->is_active())
      {
        Log::basicWarn(
            node_logging_interface_,
            "Received request for goal cancellation,"
            "but the handle is inactive, so reject the request");
        return rclcpp_action::CancelResponse::REJECT;
      }
      Log::basicInfo(
          node_logging_interface_,
          "Received request for goal cancellation");
      return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handleAccepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionType>> handle)
    {
      std::lock_guard<std::recursive_mutex> lck_guard(update_mutex_);
      Log::basicInfo(
          node_logging_interface_,
          "Receiving a new goal");

      if (isActive(current_handle_) || isRunning())
      {
        Log::basicInfo(
            node_logging_interface_,
            "An older goal is active, moving the new goal to a pending slot.");
        if (isActive(pending_handle_))
        {
          Log::basicInfo(
              node_logging_interface_,
              "The pending slot is occupied."
              " The previous pending goal will be terminated and replaced.");
          terminate(pending_handle_);
        }
        pending_handle_ = handle;
        preempt_requested_ = true;
      }
      else
      {
        if (isActive(pending_handle_))
        {
          // Shouldn't reach a state with a pending goal but no current one.
          Log::basicError(
              node_logging_interface_,
              "Forgot to handle a preemption. Terminating the pending goal.");
          terminate(pending_handle_);
          preempt_requested_ = false;
        }

        current_handle_ = handle;
        Log::basicInfo(
            node_logging_interface_,
            "Executing goal asynchronously.");
        execution_future_ = std::async(std::launch::async, [this]()
                                       { execute(); });
      }
    }

    void execute()
    {
      while (rclcpp::ok() && !stop_execution_ && isActive(current_handle_))
      {
        Log::basicInfo(
            node_logging_interface_,
            "Executing the goal...");
        try
        {
          execute_callback_();
        }
        catch (const std::exception &ex)
        {
          Log::basicError(
              node_logging_interface_,
              "Action server failed while executing action callback");
          completion_callback_();
          return;
        }

        Log::basicDebug(node_logging_interface_,
                               "Blocking process for new goal handle.");
        if (stop_execution_)
        {
          Log::basicWarn(node_logging_interface_,
                                "Stoping the thread as requested");
          terminateAll();
          completion_callback_();
          break;
        }

        if (isActive(current_handle_))
        {
          Log::basicWarn(node_logging_interface_,
                                "Current goal was not completed successfully.");
        }

        if (isActive(pending_handle_))
        {
          Log::basicWarn(node_logging_interface_,
                                "Executing a pending handle on the existing thread.");
          acceptPendingGoal();
        }
        else
        {
          Log::basicDebug(node_logging_interface_,
                                 "Done processing available goals.");
          break;
        }
      }

      Log::basicDebug(node_logging_interface_, "Action execution thread done.");
    }

    void activate()
    {
      std::lock_guard<std::recursive_mutex> lck_guard(update_mutex_);
      server_active_ = true;
      Log::basicInfo(node_logging_interface_,
                            "Activating action server...");
      if (spin_thread_)
      {
        spin_future_ = std::async(std::launch::async, [this]()
                                  { callback_group_executor_->spin(); });
      }
    }

    void deactivate()
    {
      std::lock_guard<std::recursive_mutex> lck_guard(update_mutex_);
      server_active_ = false;
      Log::basicInfo(node_logging_interface_,
                            "Deactivating action server...");
      if(spin_thread_)
      {
        // Stop executor
        callback_group_executor_->cancel();
      }
    }

    bool isRunning() const
    {
      return execution_future_.valid() &&
             (execution_future_.wait_for(std::chrono::milliseconds(0)) == std::future_status::timeout);
    }

    bool isServerActive() const
    {
      std::lock_guard<std::recursive_mutex> lck_guard(update_mutex_);
      return server_active_;
    }

    bool isPreemptRequested() const
    {
      std::lock_guard<std::recursive_mutex> lck_guard(update_mutex_);
      return preempt_requested_;
    }

    const std::shared_ptr<const typename ActionType::Goal> acceptPendingGoal()
    {
      std::lock_guard<std::recursive_mutex> lck_guard(update_mutex_);

      if (!pending_handle_ || !pending_handle_->is_active())
      {
        Log::basicError(
            node_logging_interface_,
            "Attempting to get pending goal when not available");
        return std::shared_ptr<const typename ActionType::Goal>();
      }

      if (isActive(current_handle_) && current_handle_ != pending_handle_)
      {
        Log::basicDebug(
            node_logging_interface_,
            "Cancelling the previous goal");
        current_handle_->abort(emptyResult());
      }

      current_handle_ = pending_handle_;
      pending_handle_.reset();
      preempt_requested_ = false;

      Log::basicDebug(
          node_logging_interface_,
          "Preempted goal");

      return current_handle_->get_goal();
    }

    void terminateAll(
        typename std::shared_ptr<typename ActionType::Result> result =
            std::make_shared<typename ActionType::Result>())
    {
      std::lock_guard<std::recursive_mutex> lck_guard(update_mutex_);
      terminate(current_handle_, result);
      terminate(pending_handle_, result);
      preempt_requested_ = false;
    }

    void terminatePendingGoal()
    {
      std::lock_guard<std::recursive_mutex> lck_guard(update_mutex_);

      if (!pending_handle_ || !pending_handle_->is_active())
      {
        Log::basicError(
            node_logging_interface_,
            "Attempting to terminate pending goal when not available");
        return;
      }

      terminate(pending_handle_);
      preempt_requested_ = false;

      Log::basicDebug(
          node_logging_interface_,
          "Pending goal terminated");
    }

    const std::shared_ptr<const typename ActionType::Goal> getCurrentGoal() const
    {
      std::lock_guard<std::recursive_mutex> lck_guard(update_mutex_);

      if (!isActive(current_handle_))
      {
        Log::basicError(
            node_logging_interface_,
            "A goal is not available or has reached a final state");
        return std::shared_ptr<const typename ActionType::Goal>();
      }

      return current_handle_->get_goal();
    }

    const rclcpp_action::GoalUUID getCurrentGoalID() const
    {
      std::lock_guard<std::recursive_mutex> lck_guard(update_mutex_);

      if (!isActive(current_handle_))
      {
        Log::basicError(
            node_logging_interface_,
            "A goal is not available or has reached a final state");
        return rclcpp_action::GoalUUID();
      }

      return current_handle_->get_goal_id();
    }

    const std::shared_ptr<const typename ActionType::Goal> getPendingGoal() const
    {
      std::lock_guard<std::recursive_mutex> lck_guard(update_mutex_);

      if (!pending_handle_ || !pending_handle_->is_active())
      {
        Log::basicError(
            node_logging_interface_,
            "Attempting to get pending goal when not available");
        return std::shared_ptr<const typename ActionType::Goal>();
      }

      return pending_handle_->get_goal();
    }

    bool isCancelRequested() const
    {
      std::lock_guard<std::recursive_mutex> lck_guard(update_mutex_);

      // A cancel request is assumed if either handle is canceled by the client.

      if (current_handle_ == nullptr)
      {
        Log::basicError(
            node_logging_interface_,
            "Checking for cancel but current goal is not available");
        return false;
      }

      if (pending_handle_ != nullptr)
      {
        return pending_handle_->is_canceling();
      }

      return current_handle_->is_canceling();
    }

    void terminateCurrent(
        typename std::shared_ptr<typename ActionType::Result> result =
            std::make_shared<typename ActionType::Result>())
    {
      std::lock_guard<std::recursive_mutex> lck_guard(update_mutex_);
      terminate(current_handle_, result);
    }

    void succeededCurrent(
        typename std::shared_ptr<typename ActionType::Result> result =
            std::make_shared<typename ActionType::Result>())
    {
      std::lock_guard<std::recursive_mutex> lck_guard(update_mutex_);

      if (isActive(current_handle_))
      {
        current_handle_->succeed(result);
        current_handle_.reset();
        Log::basicInfo(
            node_logging_interface_,
            "Setting succeed on current goal.");
      }
    }

    void publishFeedback(
        typename std::shared_ptr<typename ActionType::Feedback> feedback)
    {
      if (!isActive(current_handle_))
      {
        Log::basicError(
            node_logging_interface_,
            "Trying to publish feedback when the current goal handle is not active");
        return;
      }

      Log::basicInfo(node_logging_interface_,
                            "Publishing feedback");
      current_handle_->publish_feedback(feedback);
    }

  protected:
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface_;
    rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface_;
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface_;
    rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables_interface_;
    std::string action_name_;

    ExecuteCallback execute_callback_;
    CompletionCallback completion_callback_;
    std::future<void> execution_future_;
    std::future<void> spin_future_;

    typename rclcpp_action::Server<ActionType>::SharedPtr action_server_;
    typename std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionType>> current_handle_;
    typename std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionType>> pending_handle_;

    rclcpp::CallbackGroup::SharedPtr callback_group_{nullptr};
    rclcpp::executors::SingleThreadedExecutor::SharedPtr callback_group_executor_;

    mutable std::recursive_mutex update_mutex_;
    bool spin_thread_;
    bool server_active_;
    bool stop_execution_;
    bool preempt_requested_;
    std::chrono::milliseconds server_timeout_;

    bool isActive(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionType>> handle) const
    {
      return handle != nullptr && handle->is_active();
    }

    constexpr auto emptyResult() const
    {
      return std::make_shared<typename ActionType::Result>();
    }

    void terminate(
        std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionType>> handle,
        typename std::shared_ptr<typename ActionType::Result> result = std::make_shared<typename ActionType::Result>())
    {
      std::lock_guard<std::recursive_mutex> lck_guard(update_mutex_);

      if (isActive(handle))
      {
        if (handle->is_canceling())
        {
          Log::basicWarn(
              node_logging_interface_,
              "Client requested to cancel the goal. Cancelling.");
          handle->canceled(result);
        }
        else
        {
          Log::basicWarn(
              node_logging_interface_,
              "Aborting handle.");
          handle->abort(result);
        }
        handle.reset();
      }
    }
  };
} // namespace mrp_common

#endif