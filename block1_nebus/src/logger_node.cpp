// src/logger_node.cpp
// je to program (node), ktorý:
// - len počúva
// - vytvorí subscriber na topic /joint_states
// - vždy keď príde správa, vypíše polohy kĺbov do terminálu


#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class JointLogger : public rclcpp::Node
{
public:
  JointLogger() : Node("logger_node")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10,
      std::bind(&JointLogger::joint_states_callback, this, std::placeholders::_1)
    );

    // initialize last print time to now
    last_print_time_ = this->now();
  }

private:
  void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (msg->name.empty() || msg->position.empty()) return;

    // current time
    rclcpp::Time now = this->now();

    // print only every 0.5 seconds
    if ((now - last_print_time_).seconds() < 0.5)
      return;

    last_print_time_ = now;

    // Print all joints we received (joint_1, joint_2, joint_3)
    size_t n = std::min(msg->name.size(), msg->position.size());
    for (size_t i = 0; i < n; ++i)
    {
    RCLCPP_INFO(this->get_logger(),
        "%s -> %.2f",
        msg->name[i].c_str(),
        msg->position[i]);
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;

  // time of last console print
  rclcpp::Time last_print_time_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto logger = std::make_shared<JointLogger>();
  rclcpp::spin(logger);
  rclcpp::shutdown();
  return 0;
}