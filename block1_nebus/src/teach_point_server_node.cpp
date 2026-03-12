#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "block1_nebus/srv/save_point.hpp"
#include "block1_nebus/point_file_writer.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <fstream>
#include <memory>
#include <string>
#include <vector>

class TeachPointServer : public rclcpp::Node
{
public:
  TeachPointServer()
  : Node("teach_point_server_node"),
    current_positions_{0.0, 0.0, 0.0},
    have_joint_state_(false),
    point_id_(1),
  file_path_(std::string(PACKAGE_SOURCE_DIR) + std::string("/teach_points.txt")),
    file_writer_(file_path_)
  {
    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states",
      10,
      std::bind(&TeachPointServer::jointStatesCb, this, std::placeholders::_1));

    save_service_ = this->create_service<block1_nebus::srv::SavePoint>(
      "save_point",
      std::bind(&TeachPointServer::savePointCb, this,
      std::placeholders::_1, std::placeholders::_2));

    clear_service_ = this->create_service<std_srvs::srv::Trigger>(
      "clear_points",
      std::bind(&TeachPointServer::clearPointsCb, this,
      std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Teach point server started.");
    RCLCPP_INFO(this->get_logger(), "Services: /save_point, /clear_points");
    RCLCPP_INFO(this->get_logger(), "Points will be saved to %s", file_path_.c_str());
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

  void savePointCb(
    const std::shared_ptr<block1_nebus::srv::SavePoint::Request> request,
    std::shared_ptr<block1_nebus::srv::SavePoint::Response> response)
  {
    if (!have_joint_state_) {
      response->result = false;
      response->message = "No joint_states received yet.";
      RCLCPP_WARN(this->get_logger(), "Cannot save point: no joint_states received yet.");
      return;
    }

    if (request->velocity <= 0.0) {
      response->result = false;
      response->message = "Velocity must be > 0.";
      RCLCPP_WARN(this->get_logger(), "Cannot save point: invalid velocity.");
      return;
    }

    bool ok = file_writer_.savePoint(point_id_, current_positions_, request->velocity);

    if (!ok) {
      response->result = false;
      response->message = "Failed to save point to file.";
      RCLCPP_ERROR(this->get_logger(), "Failed to save point %d.", point_id_);
      return;
    }

    RCLCPP_INFO(this->get_logger(),
      "Saved point %d: [%.4f, %.4f, %.4f], v=%.4f",
      point_id_,
      current_positions_[0],
      current_positions_[1],
      current_positions_[2],
      request->velocity);

    response->result = true;
    response->message = "Point saved successfully.";
    point_id_++;
  }

  void clearPointsCb(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::ofstream file(file_path_, std::ios::trunc);
    if (!file.is_open()) {
      response->success = false;
      response->message = "Failed to clear teach_points.txt.";
      RCLCPP_ERROR(this->get_logger(), "Failed to clear teach_points.txt");
      return;
    }

    file.close();
    point_id_ = 1;

    response->success = true;
    response->message = "teach_points.txt cleared successfully.";
    RCLCPP_INFO(this->get_logger(), "teach_points.txt cleared, point numbering reset.");
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Service<block1_nebus::srv::SavePoint>::SharedPtr save_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_service_;

  std::vector<double> current_positions_;
  bool have_joint_state_;
  int point_id_;

  std::string file_path_;
  PointFileWriter file_writer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TeachPointServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}