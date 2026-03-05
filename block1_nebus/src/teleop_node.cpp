#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "rrm_msgs/srv/command.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <sstream>

using namespace std::chrono_literals;

class Teleop : public rclcpp::Node
{
public:
  Teleop() : Node("teleop_node")
  {
    client_ = this->create_client<rrm_msgs::srv::Command>("move_command");

    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
    }

    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10,
      std::bind(&Teleop::jointStatesCb, this, std::placeholders::_1));

    current_positions_ = {0.0, 0.0, 0.0};
    have_joint_state_ = false;

    RCLCPP_INFO(this->get_logger(), "Teleop service client initialized.");
    RCLCPP_INFO(this->get_logger(),
                "Enter: p1 p2 p3 max_velocity  (example: 1.0 0.5 -0.3 0.2)");
  }

  bool move(const std::vector<double>& positions, double max_velocity)
  {
    if (positions.size() != 3) {
      RCLCPP_ERROR(this->get_logger(), "positions must have size 3");
      return false;
    }
    if (max_velocity <= 0.0) {
      RCLCPP_ERROR(this->get_logger(), "max_velocity must be > 0");
      return false;
    }

    //Pockaj na aspon jeden joint_states (aby q_current nebol len 0,0,0)
    if (!waitForFirstJointState(2s)) {
      RCLCPP_WARN(this->get_logger(),
                  "No /joint_states received yet -> using [0,0,0] as current.");
    }

    //q_current = aktualne polohy klbov
    std::vector<double> q_cur = current_positions_;

    //delta = o kolko sa musi kazdy klb pohnut
    std::vector<double> delta(3);
    for (size_t i = 0; i < 3; ++i) {
      delta[i] = std::fabs(positions[i] - q_cur[i]);
    }

    double maxDelta = *std::max_element(delta.begin(), delta.end());
    if (maxDelta < 1e-9) {
      RCLCPP_INFO(this->get_logger(), "Already at target (deltas ~ 0).");
      return true;
    }

    //spolocny cas T tak, aby najdlhsi pohyb isiel rychlostou max_velocity
    double T = maxDelta / max_velocity;

    // rychlosti tak, aby vsetci dosli v case T (a nikdy nepresli max_velocity)
    std::vector<double> velocities(3);
    for (size_t i = 0; i < 3; ++i) {
      velocities[i] = delta[i] / T;
      velocities[i] = std::min(velocities[i], max_velocity);
    }

    //request na servis /move_command
    auto request = std::make_shared<rrm_msgs::srv::Command::Request>();
    request->positions = positions;
    request->velocities = velocities;

    auto future = client_->async_send_request(request);

    //pockaj na response (timeout 5s)
    auto ret = rclcpp::spin_until_future_complete(
      this->get_node_base_interface(),
      future,
      5s);

    if (ret != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Service call failed (timeout or error).");
      return false;
    }

    auto response = future.get();
    RCLCPP_INFO(this->get_logger(),
                "Service response: result_code=%d, message='%s'",
                response->result_code, response->message.c_str());

    return (response->result_code == 0);
  }

private:
  void jointStatesCb(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (msg->position.size() < 3) return;

    //Najcastejsie joint_1..joint_3, ale spravime aj fallback
    auto findIndex = [&](const std::string& joint_name) -> int {
      for (size_t i = 0; i < msg->name.size(); ++i) {
        if (msg->name[i] == joint_name) return static_cast<int>(i);
      }
      return -1;
    };

    int i1 = findIndex("joint_1");
    int i2 = findIndex("joint_2");
    int i3 = findIndex("joint_3");

    if (i1 >= 0 && i2 >= 0 && i3 >= 0 &&
        (size_t)i1 < msg->position.size() &&
        (size_t)i2 < msg->position.size() &&
        (size_t)i3 < msg->position.size())
    {
      current_positions_[0] = msg->position[i1];
      current_positions_[1] = msg->position[i2];
      current_positions_[2] = msg->position[i3];
    } else {
      current_positions_[0] = msg->position[0];
      current_positions_[1] = msg->position[1];
      current_positions_[2] = msg->position[2];
    }

    have_joint_state_ = true;
  }

  bool waitForFirstJointState(std::chrono::milliseconds timeout)
  {
    if (have_joint_state_) return true;

    auto start = this->now();
    while (rclcpp::ok() && !have_joint_state_) {
      rclcpp::spin_some(shared_from_this());
      if ((this->now() - start).seconds() * 1000.0 > timeout.count()) {
        return false;
      }
      std::this_thread::sleep_for(10ms);
    }
    return have_joint_state_;
  }

  rclcpp::Client<rrm_msgs::srv::Command>::SharedPtr client_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;

  std::vector<double> current_positions_;
  bool have_joint_state_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Teleop>();

  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);

    std::cout << "Enter p1 p2 p3 max_velocity (or 'q' to quit): ";

    std::string line;
    if (!std::getline(std::cin, line)) {
      std::cout << "End of input, exiting...\n";
      break;
    }

    // trim quick check
    if (line == "q" || line == "quit" || line == "exit") {
      std::cout << "Exiting...\n";
      break;
    }

    std::istringstream iss(line);

    double p1, p2, p3, vmax;
    std::string extra;

    //musia byt presne 4 cisla
    if (!(iss >> p1 >> p2 >> p3 >> vmax)) {
      std::cout << "Bad input! Expected 4 numbers: p1 p2 p3 max_velocity\n";
      std::cout << "Example: 1.0 0.5 -0.3 0.2\n";
      continue;
    }

    //nesmie tam byt nic navyse
    if (iss >> extra) {
      std::cout << "Bad input! Too many tokens. Use exactly 4 numbers.\n";
      continue;
    }

    //kontrola max_velocity
    if (vmax <= 0.0) {
      std::cout << "Bad input! max_velocity must be > 0.\n";
      continue;
    }

    std::vector<double> target = {p1, p2, p3};
    bool ok = node->move(target, vmax);

    std::cout << (ok ? "OK\n" : "FAILED\n");
  }

  rclcpp::shutdown();
  return 0;
}