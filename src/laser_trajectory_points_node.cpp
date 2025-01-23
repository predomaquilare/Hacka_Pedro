#include "laser_trajetory_points/laser_trajectory_points_node.hpp"

namespace laser_trajectory_points
{
    laser_trajectory_points_node::laser_trajectory_points_node(const rclcpp::NodeOptions &options) : rclcpp_lifecycle::LifecycleNode("laser_trajectory_points_node", "", options)
    {
        declare_parameter("rate.state_machine", rclcpp::ParameterValue(0.5));
        declare_parameter("waypoints.qty_points", rclcpp::ParameterValue(5.0));
        declare_parameter("waypoints.points", rclcpp::ParameterValue(std::vector<double>{0.0, 0.0, 0.0}));
    }
    laser_trajectory_points_node::~laser_trajectory_points_node() {

    };
    CallbackReturn laser_trajectory_points_node::on_configure([[maybe_unused]] const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "configurating");
        getParameters();
        configPubSub();
        configTimers();
        configService();
        configClients();

        return CallbackReturn::SUCCESS;
    }
    CallbackReturn laser_trajectory_points_node::on_activate([[maybe_unused]] const rclcpp_lifecycle::State &)
    {
        is_active = true;
        RCLCPP_INFO(get_logger(), "activating");
        goto_->on_activate();
        return CallbackReturn::SUCCESS;
    }
    CallbackReturn laser_trajectory_points_node::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State &)
    {
        is_active = false;
        RCLCPP_INFO(get_logger(), "deactivating");
        goto_->on_deactivate();
        return CallbackReturn::SUCCESS;
    }
    CallbackReturn laser_trajectory_points_node::on_cleanup([[maybe_unused]] const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "reset");
        goto_.reset();
        timer_.reset();
        return CallbackReturn::SUCCESS;
    }
    CallbackReturn laser_trajectory_points_node::on_shutdown([[maybe_unused]] const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "shutdown");
        return CallbackReturn::SUCCESS;
    }
    void laser_trajectory_points_node::getParameters()
    {
        RCLCPP_INFO(get_logger(), "configurando node");
        this->get_parameter("rate.state_machine", _rate_);
        this->get_parameter("waypoints.qty_points", _points_);
        this->get_parameter("waypoints.points", _trajectory_points_);
    }
    void laser_trajectory_points_node::configPubSub()
    {
        have_goal_ = create_subscription<std_msgs::msg::Bool>("/uav1/have_goal", 10, std::bind(&laser_trajectory_points_node::have_goal_callback, this, std::placeholders::_1));
        goto_ = create_publisher<geometry_msgs::msg::Pose>("/uav1/goto", 10);
    }
    void laser_trajectory_points_node::configTimers()
    {
        timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / _rate_), std::bind(&laser_trajectory_points_node::goto_callback, this));
    }
    void laser_trajectory_points_node::configService()
    {
        start_state_machine = create_service<std_srvs::srv::Trigger>("start_state_machine", std::bind(&laser_trajectory_points_node::start_state_machine_callback, this, std::placeholders::_1, std::placeholders::_2));
    }
    void laser_trajectory_points_node::configClients()
    {
        land_ = create_client<std_srvs::srv::Trigger>("/uav1/land");
        take_off_ = create_client<std_srvs::srv::Trigger>("/uav/takeoff");
    }

    void laser_trajectory_points_node::goto_callback()
    {
        // std::lock_guard<std::mutex> lock(mtx_);

        if (!is_active)
            take_off_command = true;
        return;
        if (!start)
            return;

        if (!have_goal)
        {
            goto_coordinates.position.x = _trajectory_points_[i * 3.0];
            goto_coordinates.position.y = _trajectory_points_[i * 3.0 + 1.0];
            goto_coordinates.position.z = _trajectory_points_[i * 3.0 + 2.0];
            goto_->publish(goto_coordinates);
            i++;
        }
        if (i > _points_)
            land_command = true;
    }

    void laser_trajectory_points_node::land_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (!is_active)
            return;
        if (!land_->service_is_ready())
        {
            return;
        }
        if (land_command)
        {
            auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
            auto result = land_->async_send_request(req);
        }
    }

    void laser_trajectory_points_node::take_off_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (!is_active)
            return;
        if (!take_off_->service_is_ready())
        {
            return;
        }
        if (take_off_command)
        {
            auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
            auto result = take_off_->async_send_request(req);
        }
    }

    void laser_trajectory_points_node::start_state_machine_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (!is_active)
            return;
        start = true;
    }

    void laser_trajectory_points_node::have_goal_callback(const std_msgs::msg::Bool::SharedPtr number)
    {
        if (!is_active)
            return;
        have_goal = number->data;
    }

} // namespace laser_trajectory_points

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(laser_trajectory_points::laser_trajectory_points_node)