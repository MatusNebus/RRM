#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "rrm_msgs/srv/command.hpp"
#include "block1_nebus/srv/play_trajectory.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <future>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

struct TeachPoint
{
  int id;
  std::vector<double> positions;
  double velocity;
};

class PlayTrajectoryServer : public rclcpp::Node
{
public:
  PlayTrajectoryServer()
  : Node("play_trajectory_server_node"),
    current_positions_{0.0, 0.0, 0.0},
    have_joint_state_(false),
    file_path_(ament_index_cpp::get_package_share_directory("block1_nebus") + std::string("/teach_points.txt"))
  {
    service_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    comm_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = comm_group_;

    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states",
      10,
      std::bind(&PlayTrajectoryServer::jointStatesCb, this, std::placeholders::_1),
      sub_options);

    move_client_ = this->create_client<rrm_msgs::srv::Command>(
      "move_command",
      rmw_qos_profile_services_default,
      comm_group_);

    while (!move_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for /move_command service.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Service /move_command not available, waiting...");
    }

    service_ = this->create_service<block1_nebus::srv::PlayTrajectory>(
      "play_trajectory",
      std::bind(&PlayTrajectoryServer::playTrajectoryCb, this,
      std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default,
      service_group_);

    RCLCPP_INFO(this->get_logger(), "Play trajectory server started.");
    RCLCPP_INFO(this->get_logger(), "Service: /play_trajectory");
    RCLCPP_INFO(this->get_logger(), "Will load teach_points from: %s", file_path_.c_str());
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

  bool loadPointsFromFile(const std::string & filename, std::vector<TeachPoint> & points)
  {
    std::ifstream file(filename);
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", filename.c_str());
      return false;
    }

    points.clear();
    std::string line;

    while (std::getline(file, line)) {
      if (line.empty()) {
        continue;
      }

      std::istringstream iss(line);
      TeachPoint point;
      double p1, p2, p3, velocity;

      if (!(iss >> point.id >> p1 >> p2 >> p3 >> velocity)) {
        RCLCPP_WARN(this->get_logger(), "Skipping invalid line: %s", line.c_str());
        continue;
      }

      point.positions = {p1, p2, p3};
      point.velocity = velocity;
      points.push_back(point);
    }

    if (points.empty()) {
      RCLCPP_WARN(this->get_logger(), "No valid points found in file.");
      return false;
    }

    return true;
  }

  bool waitForFirstJointState(std::chrono::milliseconds timeout)
  {
    auto start = std::chrono::steady_clock::now();

    while (rclcpp::ok() && !have_joint_state_) {
      if (std::chrono::steady_clock::now() - start > timeout) {
        return false;
      }
      std::this_thread::sleep_for(10ms);
    }

    return have_joint_state_;
  }

  bool sendMoveCommand(const std::vector<double> & positions, double max_velocity)
  {
    if (!waitForFirstJointState(2s)) {
      RCLCPP_ERROR(this->get_logger(), "No joint_states available.");
      return false;
    }

    if (max_velocity <= 0.0) {
      RCLCPP_ERROR(this->get_logger(), "max_velocity must be > 0.");
      return false;
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

    if (future.wait_for(timeout) != std::future_status::ready) {
      RCLCPP_ERROR(this->get_logger(), "Move command service call failed (timeout).");
      return false;
    }

    auto response = future.get();
    RCLCPP_INFO(this->get_logger(),
      "Move command response: result_code=%d, message='%s'",
      response->result_code, response->message.c_str());

    return (response->result_code == 0);
  }

  bool isTargetReached(const std::vector<double> & target)
  {
    const double tolerance = 0.01;

    return std::fabs(current_positions_[0] - target[0]) < tolerance &&
           std::fabs(current_positions_[1] - target[1]) < tolerance &&
           std::fabs(current_positions_[2] - target[2]) < tolerance;
  }

  bool waitUntilTargetReached(
    const std::vector<double> & target,
    std::chrono::milliseconds timeout)
  {
    auto start = std::chrono::steady_clock::now();

    while (rclcpp::ok()) {
      if (isTargetReached(target)) {
        return true;
      }

      if (std::chrono::steady_clock::now() - start > timeout) {
        return false;
      }

      std::this_thread::sleep_for(20ms);
    }

    return false;
  }

  void playTrajectoryCb(
    const std::shared_ptr<block1_nebus::srv::PlayTrajectory::Request> /*request*/,
    std::shared_ptr<block1_nebus::srv::PlayTrajectory::Response> response)
  {
    if (!waitForFirstJointState(2s)) {
      response->result = false;
      response->message = "No joint_states received yet.";
      RCLCPP_WARN(this->get_logger(), "Cannot play trajectory: no joint_states received yet.");
      return;
    }

    std::vector<TeachPoint> points;
    if (!loadPointsFromFile(file_path_, points)) {
      response->result = false;
      response->message = "Failed to load points from file.";
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Loaded %zu point(s). Starting playback...", points.size());

    for (const auto & point : points) {
      if (point.positions.size() != 3 || point.velocity <= 0.0) {
        response->result = false;
        response->message = "Invalid point data in file.";
        RCLCPP_ERROR(this->get_logger(), "Invalid point data for point ID %d.", point.id);
        return;
      }

      RCLCPP_INFO(this->get_logger(),
        "Sending point %d: [%.4f, %.4f, %.4f], vmax=%.4f",
        point.id,
        point.positions[0],
        point.positions[1],
        point.positions[2],
        point.velocity);

      if (!sendMoveCommand(point.positions, point.velocity)) {
        response->result = false;
        response->message = "Failed to send move command.";
        return;
      }

      if (!waitUntilTargetReached(point.positions, 30s)) {
        response->result = false;
        response->message = "Robot did not reach target in time.";
        RCLCPP_ERROR(this->get_logger(), "Timeout while waiting for point ID %d.", point.id);
        return;
      }

      RCLCPP_INFO(this->get_logger(), "Point %d reached.", point.id);
    }

    response->result = true;
    response->message = "Trajectory executed successfully.";
    RCLCPP_INFO(this->get_logger(), "Trajectory playback finished successfully.");
  }

  rclcpp::CallbackGroup::SharedPtr service_group_;
  rclcpp::CallbackGroup::SharedPtr comm_group_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Client<rrm_msgs::srv::Command>::SharedPtr move_client_;
  rclcpp::Service<block1_nebus::srv::PlayTrajectory>::SharedPtr service_;

  std::vector<double> current_positions_;
  bool have_joint_state_;
  std::string file_path_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<PlayTrajectoryServer>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}