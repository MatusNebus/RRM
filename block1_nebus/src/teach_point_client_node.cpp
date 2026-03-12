#include "rclcpp/rclcpp.hpp"
#include "block1_nebus/srv/save_point.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

using namespace std::chrono_literals;

class TeachPointClient : public rclcpp::Node
{
public:
  TeachPointClient() : Node("teach_point_client_node")
  {
    client_ = this->create_client<block1_nebus::srv::SavePoint>("save_point");

    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Service /save_point not available, waiting...");
    }

    RCLCPP_INFO(this->get_logger(), "Teach point client ready.");
  }

  bool savePoint(double velocity)
  {
    if (velocity <= 0.0) {
      RCLCPP_ERROR(this->get_logger(), "Velocity must be > 0.");
      return false;
    }

    auto request = std::make_shared<block1_nebus::srv::SavePoint::Request>();
    request->velocity = velocity;

    auto future = client_->async_send_request(request);

    auto ret = rclcpp::spin_until_future_complete(
      this->get_node_base_interface(),
      future,
      5s);

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
  rclcpp::Client<block1_nebus::srv::SavePoint>::SharedPtr client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TeachPointClient>();

  while (rclcpp::ok()) {
    std::cout << "Enter max_velocity to save current point (or 'q' to quit): ";

    std::string line;
    if (!std::getline(std::cin, line)) {
      break;
    }

    if (line == "q" || line == "quit" || line == "exit") {
      break;
    }

    std::istringstream iss(line);
    double velocity;
    std::string extra;

    if (!(iss >> velocity)) {
      std::cout << "Bad input! Enter one number.\n";
      continue;
    }

    if (iss >> extra) {
      std::cout << "Bad input! Enter exactly one number.\n";
      continue;
    }

    bool ok = node->savePoint(velocity);
    std::cout << (ok ? "POINT SAVED\n" : "FAILED\n");
  }

  rclcpp::shutdown();
  return 0;
}