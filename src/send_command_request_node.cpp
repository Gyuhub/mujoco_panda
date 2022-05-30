#include "rclcpp/rclcpp.hpp"
#include "mujoco_panda/srv/commands.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    if (argc != 7)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: send_request x y z roll pitch yaw");
        return 1;
    }
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("send_request");
    rclcpp::Client<mujoco_panda::srv::Commands>::SharedPtr client =
        node->create_client<mujoco_panda::srv::Commands>("cmd");

    auto request = std::make_shared<mujoco_panda::srv::Commands::Request>();
    request->x = atof(argv[1]);
    request->y = atof(argv[2]);
    request->z = atof(argv[3]);
    request->roll = atof(argv[4]);
    request->pitch = atof(argv[5]);
    request->yaw = atof(argv[6]);

    while (!client->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto result = client->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Response: Success? [%ld]", result.get()->finished);
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
    }

    rclcpp::shutdown();
    return 0;
}