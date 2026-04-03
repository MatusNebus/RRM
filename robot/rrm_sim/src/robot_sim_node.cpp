#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "rrm_msgs/msg/command.hpp"
#include "rrm_msgs/srv/command.hpp"
#include "rrm_sim/motion.h"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class RobotSimNode : public rclcpp::Node
{
  public:
    RobotSimNode()
    :
    Node("robot_sim"),
        joint_names_(makeJointNames(resolveJointCount())),
    robot_({0.05, 1.0}, joint_names_.size())
    {
                RCLCPP_INFO(this->get_logger(), "Configured robot with %zu active joints (%s)",
                                        joint_names_.size(), this->get_parameter("robot_name").as_string().c_str());

        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

        thread_ = std::thread([this](){
            rclcpp::Rate rate(50ms);
            while (rclcpp::ok()) {
                auto message = sensor_msgs::msg::JointState();
                message.position = robot_.getCurrentPosition();
                message.velocity = robot_.getCurrentVelocity();
                message.name = joint_names_;
                message.header.stamp = this->get_clock()->now();
                publisher_->publish(message);
                rate.sleep();
            }
        });

        subscription_ = this->create_subscription<rrm_msgs::msg::Command>(
                "move_command", 10, std::bind(&RobotSimNode::cmd_callback, this, std::placeholders::_1));

        service_ =
                this->create_service<rrm_msgs::srv::Command>("move_command", std::bind(&RobotSimNode::cmd_service_callback, this, std::placeholders::_1, std::placeholders::_2));
    }

    ~RobotSimNode() override {
        if (thread_.joinable()) {
            thread_.join();
        }
    }

  private:
    int resolveJointCount()
    {
        this->declare_parameter<std::string>("robot_name", "advancedArm");
        this->declare_parameter<int>("active_joint_count", 0);

        const auto robot_name = this->get_parameter("robot_name").as_string();
        const int configured = this->get_parameter("active_joint_count").as_int();
        if (configured > 0) {
            return configured;
        }

        if (robot_name == "simpleArm") {
            return 3;
        }

        if (robot_name == "advancedArm" || robot_name == "manipulator") {
            return 6;
        }

        RCLCPP_WARN(this->get_logger(),
                    "Unknown robot_name '%s', defaulting to 6 active joints",
                    robot_name.c_str());
        return 6;
    }

    static std::vector<std::string> makeJointNames(int count)
    {
        std::vector<std::string> names;
        names.reserve(static_cast<size_t>(count));
        for (int i = 1; i <= count; ++i) {
            names.push_back("joint_" + std::to_string(i));
        }
        return names;
    }

    std::thread thread_;
    std::vector<std::string> joint_names_;
    rrm_sim::MotorsChain robot_;
    rclcpp::Subscription<rrm_msgs::msg::Command>::SharedPtr subscription_;
    rclcpp::Service<rrm_msgs::srv::Command>::SharedPtr service_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;

    void cmd_service_callback(const std::shared_ptr<rrm_msgs::srv::Command::Request> request,
             std::shared_ptr<rrm_msgs::srv::Command::Response>      response)
    {
        std::vector<double> target_positions;
        std::vector<double> target_velocities;

        if (request->positions.size() == joint_names_.size() &&
            request->velocities.size() == joint_names_.size()) {
            target_positions = request->positions;
            target_velocities = request->velocities;
        } else if (request->positions.size() == 3 && request->velocities.size() == 3 &&
                   joint_names_.size() >= 3) {
            target_positions = robot_.getCurrentPosition();
            target_velocities.assign(joint_names_.size(), 0.1);
            for (size_t i = 0; i < 3; ++i) {
                target_positions[i] = request->positions[i];
                target_velocities[i] = request->velocities[i];
            }
        } else {
            response->result_code = 1;
            response->message =
                "Invalid command vector sizes. Expected either full joint size or first 3 joints.";
            RCLCPP_ERROR(this->get_logger(), response->message.c_str());
            return;
        }

        try {
            robot_.move(target_positions, target_velocities);
            RCLCPP_INFO(this->get_logger(), "Execution done");
            response->result_code = 0;
            response->message = "Execution done";
        } catch (std::exception &e) {
            response->result_code = 1;
            response->message = "Execution failed: " + std::string(e.what());
            RCLCPP_ERROR(this->get_logger(), response->message.c_str());
        }
    }

    void cmd_callback(const rrm_msgs::msg::Command::SharedPtr msg)
    {
        robot_.move(msg->joint_id, msg->position, 1.0);
    }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotSimNode>());
  rclcpp::shutdown();
  return 0;
}
