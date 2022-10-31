#ifndef MULTI_ROBOT_PLAYGROUND_COMMON__ACTION_CLIENT_HPP_
#define MULTI_ROBOT_PLAYGROUND_COMMON__ACTION_CLIENT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "logging.hpp"

namespace mrp_common
{
  template <typename ActionType>
  class ActionClient
  {
  public:
    typedef std::function<void(const std::shared_ptr<const typename ActionType::Feedback>)> FeedbackCallback;
    typedef std::function<void(const typename rclcpp_action::ClientGoalHandle<ActionType>::WrappedResult)> ResultCallback;

    template <typename NodeSharedPtrType>
    ActionClient(NodeSharedPtrType node,
                 const std::string &action_name,
                 FeedbackCallback feedback_callback,
                 ResultCallback result_callback,
                 bool spin_thread)
        : node_base_interface_(node->get_node_base_interface()),
          node_graph_interface_(node->get_node_graph_interface()),
          node_logging_interface_(node->get_node_logging_interface()),
          node_waitables_interface_(node->get_node_waitables_interface()),
          action_name_(action_name),
          spin_thread_(spin_thread),
          feedback_callback_(feedback_callback),
          result_callback_(result_callback)
    {
      if (spin_thread_)
      {
        callback_group_ = node_base_interface_->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        callback_group_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        callback_group_executor_->add_node(node_base_interface_);
      }
      action_client_ = rclcpp_action::create_client<ActionType>(
          node_base_interface_,
          node_graph_interface_,
          node_logging_interface_,
          node_waitables_interface_,
          action_name_, nullptr);
    }

    virtual ~ActionClient()
    {
      if (spin_thread_)
      {
        // Stop executor
        callback_group_executor_->cancel();
      }
    }

    void sendGoal(const typename ActionType::Goal &goal)
    {
      if (!action_client_->wait_for_action_server())
      {
        Log::basicError(node_logging_interface_, "Action server not available after waiting");
      }

      Log::basicInfo(node_logging_interface_, "Sending goal");
      auto send_goal_options = typename rclcpp_action::Client<ActionType>::SendGoalOptions();

      send_goal_options.goal_response_callback =
          std::bind(&ActionClient::goalResponseCallback, this, std::placeholders::_1);
      send_goal_options.feedback_callback =
          std::bind(&ActionClient::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
      send_goal_options.result_callback =
          std::bind(&ActionClient::resultCallback, this, std::placeholders::_1);
      action_client_->async_send_goal(goal, send_goal_options);

      if (spin_thread_)
      {
        // Spin executor
        spin_future_ = std::async(std::launch::async, [this]()
                                  { callback_group_executor_->spin(); });
      }
    }

    const rclcpp_action::GoalUUID getGoalID() const
    {
      std::lock_guard<std::recursive_mutex> lck_guard(client_mutex_);
      return handle_->get_goal();
    }

    const std::shared_ptr<const typename ActionType::Feedback> getFeedback() const
    {
      std::lock_guard<std::recursive_mutex> lck_guard(client_mutex_);
      return feedback_;
    }

  protected:
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface_;
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_interface_;
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface_;
    rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables_interface_;
    std::string action_name_;

    typename rclcpp_action::Client<ActionType>::SharedPtr action_client_;
    typename rclcpp_action::ClientGoalHandle<ActionType>::SharedPtr handle_;
    std::shared_ptr<typename ActionType::Feedback> feedback_;
    std::future<void> spin_future_;
    FeedbackCallback feedback_callback_;
    ResultCallback result_callback_;

    rclcpp::CallbackGroup::SharedPtr callback_group_{nullptr};
    rclcpp::executors::SingleThreadedExecutor::SharedPtr callback_group_executor_;

    mutable std::recursive_mutex client_mutex_;
    bool spin_thread_;

    void goalResponseCallback(
        std::shared_future<typename rclcpp_action::ClientGoalHandle<ActionType>::SharedPtr> goal_future)
    {
      auto goal_handle = goal_future.get();
      if (!goal_handle)
      {
        Log::basicError(node_logging_interface_, "Goal was rejected by server");
      }
      else
      {
        Log::basicInfo(node_logging_interface_, "Goal accepted by server, waiting for result");
      }
    }

    void feedbackCallback(typename rclcpp_action::ClientGoalHandle<ActionType>::SharedPtr handle,
                          const std::shared_ptr<const typename ActionType::Feedback> feedback)
    {
      Log::basicInfo(node_logging_interface_, "Receiving feedback from server");
      try
      {
        feedback_callback_(feedback);
      }
      catch (const std::exception &ex)
      {
        if (feedback_callback_ != nullptr)
        {
          Log::basicWarn(node_logging_interface_, "Failed to execute custom feedback callback");
        }
        else
        {
          Log::basicDebug(node_logging_interface_, "Feedback callback is null");
        }
      }
    }

    void resultCallback(
        const typename rclcpp_action::ClientGoalHandle<ActionType>::WrappedResult &result)
    {
      Log::basicInfo(node_logging_interface_, "Received result from server");
      switch (result.code)
      {
      case rclcpp_action::ResultCode::SUCCEEDED:
        Log::basicInfo(node_logging_interface_, "Goal was successfully completed");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        Log::basicError(node_logging_interface_, "Goal was aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        Log::basicError(node_logging_interface_, "Goal was canceled");
        break;
      default:
        Log::basicError(node_logging_interface_, "Unknown result code");
        break;
      }
      try
      {
        result_callback_(result);
      }
      catch (const std::exception &e)
      {
        if (result_callback_ != nullptr)
        {
          Log::basicWarn(node_logging_interface_, "Failed to execute custom result callback");
        }
        else
        {
          Log::basicDebug(node_logging_interface_, "Result callback is null");
        }
      }
    }
  };
}

#endif