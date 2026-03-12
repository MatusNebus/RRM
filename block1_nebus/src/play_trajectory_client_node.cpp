#include "rclcpp/rclcpp.hpp"
#include "block1_nebus/srv/play_trajectory.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <string>

using namespace std::chrono_literals;

class PlayTrajectoryClient : public rclcpp::Node
{
public:
  PlayTrajectoryClient() : Node("play_trajectory_client_node")
  {
    client_ = this->create_client<block1_nebus::srv::PlayTrajectory>("play_trajectory");

    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for /play_trajectory.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Service /play_trajectory not available, waiting...");
    }

    RCLCPP_INFO(this->get_logger(), "Play trajectory client ready.");
  }

  bool play()
  {
    auto request = std::make_shared<block1_nebus::srv::PlayTrajectory::Request>();

    auto future = client_->async_send_request(request);

    auto ret = rclcpp::spin_until_future_complete(
      this->get_node_base_interface(),
      future,
      120s);

    if (ret != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Service call failed.");
      return false;
    }

    auto response = future.get();
    RCLCPP_INFO(this->get_logger(), "Response: result=%s, message='%s'",
      response->result ? "true" : "false",
      response->message.c_str());

    return response->result;
  }

private:
  rclcpp::Client<block1_nebus::srv::PlayTrajectory>::SharedPtr client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PlayTrajectoryClient>();

  while (rclcpp::ok()) {
    std::cout << "Type 'p' to play trajectory, or 'q' to quit: ";

    std::string line;
    if (!std::getline(std::cin, line)) {
      break;
    }

    if (line == "q" || line == "quit" || line == "exit") {
      break;
    }

    if (line == "p" || line == "play") {
      bool ok = node->play();
      std::cout << (ok ? "PLAYBACK OK\n" : "PLAYBACK FAILED\n");
      continue;
    }

    std::cout << "Unknown command.\n";
  }

  rclcpp::shutdown();
  return 0;
}