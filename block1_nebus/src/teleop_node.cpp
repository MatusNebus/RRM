// src/teleop_node.cpp
// je to program (node), ktorý:
// - číta klávesnicu
// -vytvorí publisher
// - pošle správu typu rrm_msgs/msg/Command na topic /move_command
// - tým povie simulácii: “pohni kĺbom X na pozíciu Y”


#include "rclcpp/rclcpp.hpp"
#include "rrm_msgs/msg/command.hpp"

#include <iostream>   // for std::cin
#include <memory>     // for shared_ptr

class Teleop : public rclcpp::Node
{
public:
    // Constructor
    Teleop() : Node("teleop_node")
    {
        // Create publisher on topic "/move_command"
        publisher_ = this->create_publisher<rrm_msgs::msg::Command>("move_command", 10);

        RCLCPP_INFO(this->get_logger(), "Teleop node initialized.");
        RCLCPP_INFO(this->get_logger(), "Enter: joint_id position");
    }

    // Function that publishes command to robot
    void move(int joint_id, double position)
    {
        // Validate joint_id (only 0, 1, 2 allowed)
        if (joint_id != 0 && joint_id != 1 && joint_id != 2)
        {
            RCLCPP_ERROR(this->get_logger(),
                         "Invalid joint_id: %d (allowed: 0, 1, 2)",
                         joint_id);
            return;
        }

        // Create message of type rrm_msgs/msg/Command
        rrm_msgs::msg::Command msg;
        msg.joint_id = joint_id;
        msg.position = position;

        // Publish message
        publisher_->publish(msg);

        RCLCPP_INFO(this->get_logger(),
            "Sent command -> joint_id: %d, position: %.2f",
            joint_id, position);
    }

private:
    // ROS2 publisher
    rclcpp::Publisher<rrm_msgs::msg::Command>::SharedPtr publisher_;
};


// MAIN
int main(int argc, char ** argv)
{
    // Initialize ROS2
    rclcpp::init(argc, argv);

    // Create node
    auto teleop = std::make_shared<Teleop>();

    // Loop for keyboard input
    while (rclcpp::ok())
    {
        int joint_id;
        double position;

        std::cout << "Enter joint_id and position: ";

        // Read input from terminal
        std::cin >> joint_id >> position;

        if (!std::cin) {
            std::cout << "Input error, exiting...\n";
            break;
        }

        // Send command to robot
        teleop->move(joint_id, position);
    }

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}
