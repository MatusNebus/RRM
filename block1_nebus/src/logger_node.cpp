// src/logger_node.cpp
// je to program (node), ktorý:
// - len počúva
// - vytvorí subscriber na topic /joint_states
// - vždy keď príde správa, vypíše polohy kĺbov do terminálu


#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include <Eigen/Geometry>
#include <cmath>

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

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

private:
  static double deg2rad(double deg) //prevod stupňov na radiány iba
  {
    return deg * M_PI / 180.0;
  }

  static Eigen::Matrix4d dh(double a, double alpha, double d, double theta)
  {
    const double ct = std::cos(theta);
    const double st = std::sin(theta);
    const double ca = std::cos(alpha);
    const double sa = std::sin(alpha);

    Eigen::Matrix4d A;
    A << ct, -st * ca,  st * sa, a * ct,
         st,  ct * ca, -ct * sa, a * st,
         0.0,      sa,      ca,      d,
         0.0,     0.0,     0.0,    1.0;
    return A;
  }

  void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (msg->name.empty() || msg->position.empty()) return;

    auto get_joint = [&](const std::string & joint_name, double & value) -> bool {
      for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
        if (msg->name[i] == joint_name) {
          value = msg->position[i];
          return true;
        }
      }
      return false;
    };

    double q1 = 0.0;
    double q2 = 0.0;
    double q3 = 0.0;
    const bool has_q1 = get_joint("joint_1", q1);
    const bool has_q2 = get_joint("joint_2", q2);
    const bool has_q3 = get_joint("joint_3", q3);

    if (!has_q1 || !has_q2 || !has_q3) {
      RCLCPP_WARN(this->get_logger(), "Missing joint_1/joint_2/joint_3 in /joint_states.");
      return;
    }

    // DH parametre:
    // a     = [0, l2, 0]
    // alpha = [90°, 0°, 90°]
    // d     = [l1 + l0, 0, 0]
    // theta = [180° + q1, 90° + q2, 90° + q3]
    const Eigen::Matrix4d A1 = dh(0.0, deg2rad(90.0), l1_ + l0_, deg2rad(180.0) + q1);
    const Eigen::Matrix4d A2 = dh(l2_, deg2rad(0.0), 0.0, deg2rad(90.0) + q2);
    const Eigen::Matrix4d A3 = dh(0.0, deg2rad(90.0), 0.0, deg2rad(90.0) + q3);
    const Eigen::Matrix4d T03 = A1 * A2 * A3;

    // Fixed offset from frame 3 to tool0: translation along local Z axis
    Eigen::Matrix4d T3_tool0 = Eigen::Matrix4d::Identity();
    T3_tool0(2, 3) = 0.203;
    const Eigen::Matrix4d T0_tool0 = T03 * T3_tool0;

    const Eigen::Matrix3d R03 = T0_tool0.block<3, 3>(0, 0);
    const Eigen::Quaterniond q_tcp(R03);

    const double x = T0_tool0(0, 3);
    const double y = T0_tool0(1, 3);
    const double z = T0_tool0(2, 3);

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = msg->header.stamp;
    tf_msg.header.frame_id = "base_link";
    tf_msg.child_frame_id = "fk_tool0";
    tf_msg.transform.translation.x = x;
    tf_msg.transform.translation.y = y;
    tf_msg.transform.translation.z = z;
    tf_msg.transform.rotation.x = q_tcp.x();
    tf_msg.transform.rotation.y = q_tcp.y();
    tf_msg.transform.rotation.z = q_tcp.z();
    tf_msg.transform.rotation.w = q_tcp.w();
    tf_broadcaster_->sendTransform(tf_msg);

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

    RCLCPP_INFO(this->get_logger(),
      "FK TCP position: x=%.4f, y=%.4f, z=%.4f",
      x, y, z);
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // time of last console print
  rclcpp::Time last_print_time_;

  // Robot geometric constants used in DH parameters.
  const double l1_ = 0.0;
  const double l0_ = 0.0;
  const double l2_ = 0.203;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto logger = std::make_shared<JointLogger>();
  rclcpp::spin(logger);
  rclcpp::shutdown();
  return 0;
}