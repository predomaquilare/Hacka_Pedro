#include "laser_trajectory_points_node.cpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<laser_trajectory_points::laser_trajectory_points_node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}