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

    geometry_msgs::msg::Pose goto_coordinates;

    std::mutex mtx_;

    void have_goal_callback(const std_msgs::msg::Bool::SharedPtr number);
    void start_state_machine_callback(std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void land_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void take_off_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void goto_callback();

    bool is_active;
    void getParameters();
    void configPubSub();
    void configTimers();
    void configService();
    void configClients();

    float _points_;
    std::vector<double> _trajectory_points_;
    float _rate_;
    bool have_goal;
    bool start = false;
    bool land_command = false;
    bool take_off_command = false;
    int i = 0;
  };

} // namespace laser_trajectory_points
