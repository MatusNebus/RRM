#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "rrm_msgs/srv/command.hpp"
#include "block1_nebus/srv/save_point.hpp"
#include "block1_nebus/srv/play_trajectory.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

class ControlNode : public rclcpp::Node
{
public:
  ControlNode()
  : Node("control_node"),
    current_positions_{0.0, 0.0, 0.0},
    have_joint_state_(false)
  {
    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10,
      std::bind(&ControlNode::jointStatesCb, this, std::placeholders::_1));

    move_client_ = this->create_client<rrm_msgs::srv::Command>("move_command");
    save_client_ = this->create_client<block1_nebus::srv::SavePoint>("save_point");
    play_client_ = this->create_client<block1_nebus::srv::PlayTrajectory>("play_trajectory");
    clear_client_ = this->create_client<std_srvs::srv::Trigger>("clear_points");

    waitForService(move_client_, "/move_command");
    waitForService(save_client_, "/save_point");
    waitForService(play_client_, "/play_trajectory");
    waitForService(clear_client_, "/clear_points");

    RCLCPP_INFO(this->get_logger(), "Control node ready.");
  }

  bool move(const std::vector<double> & positions, double max_velocity)
{
  if (positions.size() != 3) {
    RCLCPP_ERROR(this->get_logger(), "positions must have size 3");
    return false;
  }

  if (max_velocity <= 0.0) {
    RCLCPP_ERROR(this->get_logger(), "max_velocity must be > 0");
    return false;
  }

  if (!waitForFirstJointState(2s)) {
    RCLCPP_WARN(this->get_logger(),
      "No /joint_states received yet -> using [0,0,0] as current.");
  }

  std::vector<double> q_cur = current_positions_;

  std::vector<double> delta(3);
  for (size_t i = 0; i < 3; ++i) {
    delta[i] = std::fabs(positions[i] - q_cur[i]);
  }

  double maxDelta = *std::max_element(delta.begin(), delta.end());
  if (maxDelta < 1e-9) {
    RCLCPP_INFO(this->get_logger(), "Already at target.");
    return true;
  }

  double T = maxDelta / max_velocity;

  std::vector<double> velocities(3);
  for (size_t i = 0; i < 3; ++i) {
    velocities[i] = delta[i] / T;
    velocities[i] = std::min(velocities[i], max_velocity);
  }

  auto request = std::make_shared<rrm_msgs::srv::Command::Request>();
  request->positions = positions;
  request->velocities = velocities;

  auto future = move_client_->async_send_request(request);

  auto timeout = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::duration<double>(T + 2.0));

  auto ret = rclcpp::spin_until_future_complete(
    this->get_node_base_interface(),
    future,
    timeout);

  if (ret != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Move service call failed (timeout or error).");
    return false;
  }

  auto response = future.get();
  RCLCPP_INFO(this->get_logger(),
    "Move response: result_code=%d, message='%s'",
    response->result_code, response->message.c_str());

  return (response->result_code == 0);
}

  bool savePoint(double velocity)
  {
    if (velocity <= 0.0) {
      RCLCPP_ERROR(this->get_logger(), "Velocity must be > 0.");
      return false;
    }

    auto request = std::make_shared<block1_nebus::srv::SavePoint::Request>();
    request->velocity = velocity;

    auto future = save_client_->async_send_request(request);

    auto ret = rclcpp::spin_until_future_complete(
      this->get_node_base_interface(),
      future,
      5s);

    if (ret != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Save point service call failed.");
      return false;
    }

    auto response = future.get();
    RCLCPP_INFO(this->get_logger(), "Save response: result=%s, message='%s'",
      response->result ? "true" : "false",
      response->message.c_str());

    return response->result;
  }

  bool playTrajectory()
  {
    auto request = std::make_shared<block1_nebus::srv::PlayTrajectory::Request>();

    auto future = play_client_->async_send_request(request);

    auto ret = rclcpp::spin_until_future_complete(
      this->get_node_base_interface(),
      future,
      120s);

    if (ret != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Play trajectory service call failed.");
      return false;
    }

    auto response = future.get();
    RCLCPP_INFO(this->get_logger(), "Play response: result=%s, message='%s'",
      response->result ? "true" : "false",
      response->message.c_str());

    return response->result;
  }

  bool clearPoints()
  {
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

    auto future = clear_client_->async_send_request(request);

    auto ret = rclcpp::spin_until_future_complete(
      this->get_node_base_interface(),
      future,
      5s);

    if (ret != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Clear points service call failed.");
      return false;
    }

    auto response = future.get();
    RCLCPP_INFO(this->get_logger(), "Clear response: success=%s, message='%s'",
      response->success ? "true" : "false",
      response->message.c_str());

    return response->success;
  }

private:
  void jointStatesCb(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (msg->position.size() < 3) {
      return;
    }

    auto findIndex = [&](const std::string & joint_name) -> int {
      for (size_t i = 0; i < msg->name.size(); ++i) {
        if (msg->name[i] == joint_name) {
          return static_cast<int>(i);
        }
      }
      return -1;
    };

    int i1 = findIndex("joint_1");
    int i2 = findIndex("joint_2");
    int i3 = findIndex("joint_3");

    if (i1 >= 0 && i2 >= 0 && i3 >= 0 &&
        static_cast<size_t>(i1) < msg->position.size() &&
        static_cast<size_t>(i2) < msg->position.size() &&
        static_cast<size_t>(i3) < msg->position.size())
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
    if (have_joint_state_) {
      return true;
    }

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

  template<typename ClientT>
  void waitForService(const ClientT & client, const std::string & service_name)
  {
    while (!client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for %s", service_name.c_str());
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Service %s not available, waiting...", service_name.c_str());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;

  rclcpp::Client<rrm_msgs::srv::Command>::SharedPtr move_client_;
  rclcpp::Client<block1_nebus::srv::SavePoint>::SharedPtr save_client_;
  rclcpp::Client<block1_nebus::srv::PlayTrajectory>::SharedPtr play_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr clear_client_;

  std::vector<double> current_positions_;
  bool have_joint_state_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ControlNode>();

  while (rclcpp::ok()) {
    rclcpp::spin_some(node);

    std::cout << "\n===== CONTROL MENU =====\n";
    std::cout << "m - move robot\n";
    std::cout << "s - save current point\n";
    std::cout << "p - play saved trajectory\n";
    std::cout << "c - clear teach_points.txt\n";
    std::cout << "q - quit\n";
    std::cout << "Choose: ";

    std::string cmd;
    if (!std::getline(std::cin, cmd)) {
      break;
    }

    if (cmd == "q") {
      break;
    }

    if (cmd == "m") {
      std::cout << "Enter p1 p2 p3 max_velocity: ";
      std::string line;
      if (!std::getline(std::cin, line)) {
        break;
      }

      std::istringstream iss(line);
      double p1, p2, p3, vmax;
      std::string extra;

      if (!(iss >> p1 >> p2 >> p3 >> vmax)) {
        std::cout << "Bad input.\n";
        continue;
      }

      if (iss >> extra) {
        std::cout << "Too many tokens.\n";
        continue;
      }

      bool ok = node->move({p1, p2, p3}, vmax);
      std::cout << (ok ? "MOVE OK\n" : "MOVE FAILED\n");
      continue;
    }

    if (cmd == "s") {
      std::cout << "Enter velocity for current point: ";
      std::string line;
      if (!std::getline(std::cin, line)) {
        break;
      }

      std::istringstream iss(line);
      double velocity;
      std::string extra;

      if (!(iss >> velocity)) {
        std::cout << "Bad input.\n";
        continue;
      }

      if (iss >> extra) {
        std::cout << "Too many tokens.\n";
        continue;
      }

      bool ok = node->savePoint(velocity);
      std::cout << (ok ? "SAVE OK\n" : "SAVE FAILED\n");
      continue;
    }

    if (cmd == "p") {
      bool ok = node->playTrajectory();
      std::cout << (ok ? "PLAY OK\n" : "PLAY FAILED\n");
      continue;
    }

    if (cmd == "c") {
      std::cout << "Are you sure? This will delete all saved points. (y/n): ";
      std::string confirm;
      if (!std::getline(std::cin, confirm)) {
        break;
      }

      if (confirm == "y" || confirm == "Y") {
        bool ok = node->clearPoints();
        std::cout << (ok ? "CLEAR OK\n" : "CLEAR FAILED\n");
      } else {
        std::cout << "Cancelled.\n";
      }
      continue;
    }

    std::cout << "Unknown command.\n";
  }

  rclcpp::shutdown();
  return 0;
}