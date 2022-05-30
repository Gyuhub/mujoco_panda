#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <stdlib.h>

int main(int argc, char * argv[])
{
    if (argc != 7)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Usage: x y z r p y");
        return -1;
    }
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("send_topic");
    auto pub = node->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/hqp_controller/commands",
        10);
    auto msg = std_msgs::msg::Float64MultiArray();
    for (size_t i = 0; i < 6; i++) msg.data.push_back(atof(argv[i + 1]));
    RCLCPP_INFO(node->get_logger(), "Publish a command: %f %f %f %f %f %f",
                msg.data[0],
                msg.data[1],
                msg.data[2],
                msg.data[3],
                msg.data[4],
                msg.data[5]);
    pub->publish(msg);

    rclcpp::shutdown();
    return 0;
}