#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace laser_trajectory_points
{
  class laser_trajectory_points_node : public rclcpp_lifecycle::LifecycleNode
  {
  public:
    explicit laser_trajectory_points_node(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~laser_trajectory_points_node() override;

  private:
    CallbackReturn on_configure(const rclcpp_lifecycle::State &);
    CallbackReturn on_activate(const rclcpp_lifecycle::State &state);
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state);
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state);

    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Pose>::SharedPtr goto_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr have_goal_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr land_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr take_off_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_state_machine;
    rclcpp::TimerBase::SharedPtr timer_;

    std::mutex mtx_;

    void goto_callback();
    void have_goal_callback(const std_msgs::msg::Bool::SharedPtr number);
    void start_state_machine_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    bool is_active;
    void getParameters();
    void configPubSub();
    void configTimers();
    void configService();
    void configClients();

    std::vector<_Float64> _trajectory_points_;
    int _points_;
    float _rate_;
  };

} // namespace laser_trajectory_points
