#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter_client.hpp"

#include "sensor_msgs/msg/joint_state.hpp"

#include "block1_nebus/srv/get_ik_solutions6_dof.hpp"
#include "block1_nebus/srv/get_best_ik_solution6_dof.hpp"

#include <urdf/model.h>
#include <Eigen/Geometry>

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

class IkSolverNode : public rclcpp::Node
{
public:
  IkSolverNode() : Node("ik_solver_6dof_node")
  {
    // Subscriber na aktualny stav robota
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&IkSolverNode::joint_states_callback, this, std::placeholders::_1)
    );

    // Service pre vypocet vsetkych validnych IK rieseni
    get_all_solutions_srv_ =
      this->create_service<block1_nebus::srv::GetIkSolutions6Dof>(
        "get_ik_solutions_6dof",
        std::bind(
          &IkSolverNode::handle_get_ik_solutions,
          this,
          std::placeholders::_1,
          std::placeholders::_2
        )
      );

    // Service pre vypocet najlepsieho IK riesenia
    get_best_solution_srv_ =
      this->create_service<block1_nebus::srv::GetBestIkSolution6Dof>(
        "get_best_ik_solution_6dof",
        std::bind(
          &IkSolverNode::handle_get_best_ik_solution,
          this,
          std::placeholders::_1,
          std::placeholders::_2
        )
      );

    // Pri starte sa pokusime nacitat joint limity z URDF
    for (int attempt = 1; attempt <= 10 && !has_joint_limits_; ++attempt) {
      if (load_joint_limits_from_urdf()) {
        break;
      }

      if (attempt < 10) {
        RCLCPP_WARN(
          this->get_logger(),
          "Failed to load joint limits from URDF (attempt %d/10). Retrying...",
          attempt
        );
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }
    }

    if (!has_joint_limits_) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Joint limits were not loaded during startup. IK services will reject requests until restart."
      );
    }

    RCLCPP_INFO(this->get_logger(), "ik_solver_6dof_node started.");
  }

private:
  struct JointLimits
  {
    double lower{0.0};
    double upper{0.0};
    bool valid{false};
  };

  struct JointSolution
  {
    double q1{0.0};
    double q2{0.0};
    double q3{0.0};
    double q4{0.0};
    double q5{0.0};
    double q6{0.0};
  };

  // =========================================================
  // 1. CALLBACK NA /joint_states
  // =========================================================

  void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (msg->name.empty() || msg->position.empty()) {
      return;
    }

    double q1 = 0.0;
    double q2 = 0.0;
    double q3 = 0.0;
    double q4 = 0.0;
    double q5 = 0.0;
    double q6 = 0.0;

    bool has_q1 = get_joint_value(*msg, "joint_1", q1);
    bool has_q2 = get_joint_value(*msg, "joint_2", q2);
    bool has_q3 = get_joint_value(*msg, "joint_3", q3);
    bool has_q4 = get_joint_value(*msg, "joint_4", q4);
    bool has_q5 = get_joint_value(*msg, "joint_5", q5);
    bool has_q6 = get_joint_value(*msg, "joint_6", q6);

    if (!has_q1 || !has_q2 || !has_q3 || !has_q4 || !has_q5 || !has_q6) {
      return;
    }

    current_q1_ = normalize_angle(q1);
    current_q2_ = normalize_angle(q2);
    current_q3_ = normalize_angle(q3);
    current_q4_ = normalize_angle(q4);
    current_q5_ = normalize_angle(q5);
    current_q6_ = normalize_angle(q6);
    has_current_joint_state_ = true;
  }

  bool get_joint_value(
    const sensor_msgs::msg::JointState & msg,
    const std::string & joint_name,
    double & value) const
  {
    size_t n = std::min(msg.name.size(), msg.position.size());

    for (size_t i = 0; i < n; ++i) {
      if (msg.name[i] == joint_name) {
        value = msg.position[i];
        return true;
      }
    }
    return false;
  }

  // =========================================================
  // 2. POMOCNE MATEMATICKE FUNKCIE
  // =========================================================

  double normalize_angle(double angle) const
  {
    while (angle > M_PI) {
      angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
      angle += 2.0 * M_PI;
    }
    return angle;
  }

  double clamp_to_unit_interval(double value) const
  {
    if (value > 1.0) {
      return 1.0;
    }
    if (value < -1.0) {
      return -1.0;
    }
    return value;
  }

  // Quaternion -> rotacna matica
  Eigen::Matrix3d quaternion_to_rotation_matrix(
    double qx, double qy, double qz, double qw) const
  {
    Eigen::Quaterniond q(qw, qx, qy, qz);
    q.normalize();
    return q.toRotationMatrix();
  }

  // Rotacia prvych troch klbov.
  // Pre nas model plati:
  // R03 = Rz(q1) * Ry(q2) * Ry(q3) = Rz(q1) * Ry(q2 + q3)
  Eigen::Matrix3d compute_R03(double q1, double q2, double q3) const
  {
    Eigen::AngleAxisd rz(q1, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd ry2(q2, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd ry3(q3, Eigen::Vector3d::UnitY());

    Eigen::Matrix3d R03 = (rz * ry2 * ry3).toRotationMatrix();
    return R03;
  }

  // Pomocna funkcia: prida riesenie, ak este nie je duplicitne
  void add_solution_if_unique(
    std::vector<JointSolution> & solutions,
    const JointSolution & sol) const
  {
    for (const auto & existing : solutions) {
      const double dq1 = std::abs(normalize_angle(sol.q1 - existing.q1));
      const double dq2 = std::abs(normalize_angle(sol.q2 - existing.q2));
      const double dq3 = std::abs(normalize_angle(sol.q3 - existing.q3));
      const double dq4 = std::abs(normalize_angle(sol.q4 - existing.q4));
      const double dq5 = std::abs(normalize_angle(sol.q5 - existing.q5));
      const double dq6 = std::abs(normalize_angle(sol.q6 - existing.q6));

      if (dq1 < 1e-6 && dq2 < 1e-6 && dq3 < 1e-6 &&
          dq4 < 1e-6 && dq5 < 1e-6 && dq6 < 1e-6) {
        return;
      }
    }

    solutions.push_back(sol);
  }

  // =========================================================
  // 3. NACITANIE JOINT LIMITOV Z URDF
  // =========================================================

  bool load_joint_limits_from_urdf()
  {
    auto param_client =
      std::make_shared<rclcpp::SyncParametersClient>(this, "robot_state_publisher");

    if (!param_client->wait_for_service(std::chrono::seconds(3))) {
      RCLCPP_WARN(
        this->get_logger(),
        "Parameter service of robot_state_publisher is not available. Joint limits were not loaded."
      );
      return false;
    }

    auto params = param_client->get_parameters({"robot_description"});

    if (params.empty()) {
      RCLCPP_WARN(this->get_logger(), "Parameter robot_description was not found.");
      return false;
    }

    std::string urdf_xml = params[0].as_string();

    if (urdf_xml.empty()) {
      RCLCPP_WARN(this->get_logger(), "robot_description is empty.");
      return false;
    }

    urdf::Model model;
    bool ok = model.initString(urdf_xml);

    if (!ok) {
      RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF from robot_description.");
      return false;
    }

    bool ok1 = extract_joint_limits(model, "joint_1", joint1_limits_);
    bool ok2 = extract_joint_limits(model, "joint_2", joint2_limits_);
    bool ok3 = extract_joint_limits(model, "joint_3", joint3_limits_);
    bool ok4 = extract_joint_limits(model, "joint_4", joint4_limits_);
    bool ok5 = extract_joint_limits(model, "joint_5", joint5_limits_);
    bool ok6 = extract_joint_limits(model, "joint_6", joint6_limits_);

    if (!ok1 || !ok2 || !ok3 || !ok4 || !ok5 || !ok6) {
      RCLCPP_WARN(this->get_logger(), "Some joint limits could not be loaded from URDF.");
      return false;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Loaded joint limits successfully for joints 1..6."
    );

    has_joint_limits_ = true;
    return true;
  }

  bool extract_joint_limits(
    const urdf::Model & model,
    const std::string & joint_name,
    JointLimits & limits)
  {
    auto joint = model.getJoint(joint_name);

    if (!joint) {
      RCLCPP_WARN(this->get_logger(), "Joint '%s' was not found in URDF.", joint_name.c_str());
      return false;
    }

    if (!joint->limits) {
      RCLCPP_WARN(this->get_logger(), "Joint '%s' has no limits in URDF.", joint_name.c_str());
      return false;
    }

    limits.lower = joint->limits->lower;
    limits.upper = joint->limits->upper;
    limits.valid = true;
    return true;
  }

  bool is_within_limits(double value, const JointLimits & limits) const
  {
    if (!limits.valid) {
      return false;
    }

    const double normalized = normalize_angle(value);
    return (normalized >= limits.lower && normalized <= limits.upper);
  }

  bool is_valid_solution(const JointSolution & solution) const
  {
    if (!has_joint_limits_) {
      return false;
    }

    return is_within_limits(solution.q1, joint1_limits_) &&
           is_within_limits(solution.q2, joint2_limits_) &&
           is_within_limits(solution.q3, joint3_limits_) &&
           is_within_limits(solution.q4, joint4_limits_) &&
           is_within_limits(solution.q5, joint5_limits_) &&
           is_within_limits(solution.q6, joint6_limits_);
  }

  // =========================================================
  // 4. VYBER "BEST SOLUTION"
  // =========================================================

  double joint_distance_to_current(const JointSolution & solution) const
  {
    if (!has_current_joint_state_) {
      return std::numeric_limits<double>::infinity();
    }

    const double dq1 = normalize_angle(solution.q1 - current_q1_);
    const double dq2 = normalize_angle(solution.q2 - current_q2_);
    const double dq3 = normalize_angle(solution.q3 - current_q3_);
    const double dq4 = normalize_angle(solution.q4 - current_q4_);
    const double dq5 = normalize_angle(solution.q5 - current_q5_);
    const double dq6 = normalize_angle(solution.q6 - current_q6_);

    return std::sqrt(
      dq1 * dq1 + dq2 * dq2 + dq3 * dq3 +
      dq4 * dq4 + dq5 * dq5 + dq6 * dq6
    );
  }

  // =========================================================
  // 5. ANALYTICKY IK SOLVER - VSETKY KANDIDATNE RIESENIA
  // =========================================================
  //
  // Bonus riesime metodou kinematic decoupling:
  //
  // 1. Zo zadanej polohy a orientacie toolu vypocitame wrist center:
  //    oc = o - R * [0 0 d6]^T
  //
  // 2. Z polohy wrist center vyriesime prve 3 klby:
  //    q1, q2, q3
  //
  // 3. Potom spocitame:
  //    R36 = R03^T * R06
  //
  // 4. Z R36 vyriesime posledne 3 klby zapastia:
  //    q4, q5, q6
  //
  std::vector<JointSolution> compute_all_ik_solutions(
    double x, double y, double z,
    double qx, double qy, double qz, double qw)
  {
    std::vector<JointSolution> solutions;

    // Zadana orientacia toolu
    const Eigen::Matrix3d R06 = quaternion_to_rotation_matrix(qx, qy, qz, qw);

    // Pozicia toolu
    const Eigen::Vector3d o(x, y, z);

    // Wrist center:
    // tool je posunuty od stredu zapastia o d6 po osi z6
    const Eigen::Vector3d oc = o - d6_ * R06.col(2);

    // -----------------------------------------------------
    // CAST A: IK polohy wrist center -> q1, q2, q3
    // -----------------------------------------------------

    // Pre prvu cast mame rovnaku geometriu ako pri 3DOF,
    // len tretia efektivna dlzka je:
    // L3w = l3 + l4
    const double x_wc = oc.x();
    const double y_wc = oc.y();
    const double z_wc = oc.z();

    const double d = l0_ + l1_;
    const double l2 = l2_;
    const double l3w = l3_ + l4_;

    const double rho = std::sqrt(x_wc * x_wc + y_wc * y_wc);
    const double z_local = z_wc - d;

    const double reach_sq = rho * rho + z_local * z_local;
    const double raw_cos_q3 = (reach_sq - l2 * l2 - l3w * l3w) / (2.0 * l2 * l3w);

    const double eps = 1e-9;
    if (raw_cos_q3 < -1.0 - eps || raw_cos_q3 > 1.0 + eps) {
      return solutions;
    }

    const double cos_q3 = clamp_to_unit_interval(raw_cos_q3);

    const double q3_a = std::acos(cos_q3);
    const double q3_b = -std::acos(cos_q3);

    const double q1_base = std::atan2(y_wc, x_wc);

    const double q1_candidates[2] = {
      q1_base,
      q1_base + M_PI
    };

    const double rho_candidates[2] = {
      rho,
      -rho
    };

    const double q3_candidates[2] = {
      q3_a,
      q3_b
    };

    // -----------------------------------------------------
    // CAST B: IK orientacie zapastia -> q4, q5, q6
    // -----------------------------------------------------
    //
    // Zapastie modelujeme ako:
    // R36 = Rz(q4) * Ry(q5) * Rz(q6)
    //
    for (int i = 0; i < 2; ++i) {
      const double q1 = q1_candidates[i];
      const double rho_signed = rho_candidates[i];

      for (int j = 0; j < 2; ++j) {
        const double q3 = q3_candidates[j];

        const double k1 = l2 + l3w * std::cos(q3);
        const double k2 = l3w * std::sin(q3);

        const double q2 =
          std::atan2(rho_signed, z_local) - std::atan2(k2, k1);

        const double q1n = normalize_angle(q1);
        const double q2n = normalize_angle(q2);
        const double q3n = normalize_angle(q3);

        // Rotacia prvej casti robota
        const Eigen::Matrix3d R03 = compute_R03(q1n, q2n, q3n);

        // Rotacia zapastia
        const Eigen::Matrix3d R36 = R03.transpose() * R06;

        const double r13 = R36(0, 2);
        const double r23 = R36(1, 2);
        const double r31 = R36(2, 0);
        const double r32 = R36(2, 1);
        const double r33 = R36(2, 2);

        // Nesingularny pripad
        const double s5_abs = std::sqrt(std::max(0.0, 1.0 - r33 * r33));

        if (s5_abs > 1e-8) {
          // Riesenie 1: s5 > 0
          {
            JointSolution sol;
            sol.q1 = q1n;
            sol.q2 = q2n;
            sol.q3 = q3n;
            sol.q5 = normalize_angle(std::atan2(s5_abs, r33));
            sol.q4 = normalize_angle(std::atan2(r23, r13));
            sol.q6 = normalize_angle(std::atan2(r32, -r31));
            add_solution_if_unique(solutions, sol);
          }

          // Riesenie 2: s5 < 0
          {
            JointSolution sol;
            sol.q1 = q1n;
            sol.q2 = q2n;
            sol.q3 = q3n;
            sol.q5 = normalize_angle(std::atan2(-s5_abs, r33));
            sol.q4 = normalize_angle(std::atan2(-r23, -r13));
            sol.q6 = normalize_angle(std::atan2(-r32, r31));
            add_solution_if_unique(solutions, sol);
          }
        } else {
          // Singularny pripad:
          // q5 je 0 alebo pi a q4,q6 nie su urcene jednoznacne.
          // Zvolime jednoduche reprezentativne riesenie q4 = 0.
          if (r33 > 0.0) {
            // q5 = 0  -> R36 = Rz(q4 + q6)
            JointSolution sol;
            sol.q1 = q1n;
            sol.q2 = q2n;
            sol.q3 = q3n;
            sol.q4 = 0.0;
            sol.q5 = 0.0;
            sol.q6 = normalize_angle(std::atan2(R36(1, 0), R36(0, 0)));
            add_solution_if_unique(solutions, sol);
          } else {
            // q5 = pi -> reprezentativna volba q4 = 0
            JointSolution sol;
            sol.q1 = q1n;
            sol.q2 = q2n;
            sol.q3 = q3n;
            sol.q4 = 0.0;
            sol.q5 = M_PI;
            sol.q6 = normalize_angle(std::atan2(R36(1, 0), -R36(0, 0)));
            add_solution_if_unique(solutions, sol);
          }
        }
      }
    }

    return solutions;
  }

  std::vector<JointSolution> compute_valid_ik_solutions(
    double x, double y, double z,
    double qx, double qy, double qz, double qw)
  {
    std::vector<JointSolution> all_solutions =
      compute_all_ik_solutions(x, y, z, qx, qy, qz, qw);

    std::vector<JointSolution> valid_solutions;

    for (const auto & sol : all_solutions) {
      JointSolution normalized_sol;
      normalized_sol.q1 = normalize_angle(sol.q1);
      normalized_sol.q2 = normalize_angle(sol.q2);
      normalized_sol.q3 = normalize_angle(sol.q3);
      normalized_sol.q4 = normalize_angle(sol.q4);
      normalized_sol.q5 = normalize_angle(sol.q5);
      normalized_sol.q6 = normalize_angle(sol.q6);

      if (is_valid_solution(normalized_sol)) {
        valid_solutions.push_back(normalized_sol);
      }
    }

    return valid_solutions;
  }

  // =========================================================
  // 6. SERVICE: VSETKY VALIDNE RIESENIA
  // =========================================================

  void handle_get_ik_solutions(
    const std::shared_ptr<block1_nebus::srv::GetIkSolutions6Dof::Request> request,
    std::shared_ptr<block1_nebus::srv::GetIkSolutions6Dof::Response> response)
  {
    if (!has_joint_limits_) {
      response->success = false;
      response->message = "Joint limits are not available.";
      response->solution_count = 0;
      return;
    }

    std::vector<JointSolution> valid_solutions =
      compute_valid_ik_solutions(
        request->x, request->y, request->z,
        request->qx, request->qy, request->qz, request->qw
      );

    response->solution_count = static_cast<uint32_t>(valid_solutions.size());

    if (valid_solutions.empty()) {
      response->success = false;
      response->message = "No valid IK solution found.";
      return;
    }

    for (const auto & sol : valid_solutions) {
      response->q1.push_back(sol.q1);
      response->q2.push_back(sol.q2);
      response->q3.push_back(sol.q3);
      response->q4.push_back(sol.q4);
      response->q5.push_back(sol.q5);
      response->q6.push_back(sol.q6);
    }

    response->success = true;
    response->message = "Valid IK solutions found.";
  }

  // =========================================================
  // 7. SERVICE: NAJLEPSIE RIESENIE
  // =========================================================

  void handle_get_best_ik_solution(
    const std::shared_ptr<block1_nebus::srv::GetBestIkSolution6Dof::Request> request,
    std::shared_ptr<block1_nebus::srv::GetBestIkSolution6Dof::Response> response)
  {
    if (!has_joint_limits_) {
      response->success = false;
      response->message = "Joint limits are not available.";
      return;
    }

    if (!has_current_joint_state_) {
      response->success = false;
      response->message = "Current joint state is not available yet.";
      return;
    }

    std::vector<JointSolution> valid_solutions =
      compute_valid_ik_solutions(
        request->x, request->y, request->z,
        request->qx, request->qy, request->qz, request->qw
      );

    if (valid_solutions.empty()) {
      response->success = false;
      response->message = "No valid IK solution found.";
      return;
    }

    size_t best_index = 0;
    double best_distance = joint_distance_to_current(valid_solutions[0]);

    for (size_t i = 1; i < valid_solutions.size(); ++i) {
      double dist = joint_distance_to_current(valid_solutions[i]);
      if (dist < best_distance) {
        best_distance = dist;
        best_index = i;
      }
    }

    const auto & best = valid_solutions[best_index];

    response->q1 = best.q1;
    response->q2 = best.q2;
    response->q3 = best.q3;
    response->q4 = best.q4;
    response->q5 = best.q5;
    response->q6 = best.q6;
    response->success = true;
    response->message = "Best IK solution found.";
  }

  // =========================================================
  // 8. CLENSKE PREMENNE
  // =========================================================

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  rclcpp::Service<block1_nebus::srv::GetIkSolutions6Dof>::SharedPtr get_all_solutions_srv_;
  rclcpp::Service<block1_nebus::srv::GetBestIkSolution6Dof>::SharedPtr get_best_solution_srv_;

  JointLimits joint1_limits_;
  JointLimits joint2_limits_;
  JointLimits joint3_limits_;
  JointLimits joint4_limits_;
  JointLimits joint5_limits_;
  JointLimits joint6_limits_;
  bool has_joint_limits_{false};

  double current_q1_{0.0};
  double current_q2_{0.0};
  double current_q3_{0.0};
  double current_q4_{0.0};
  double current_q5_{0.0};
  double current_q6_{0.0};
  bool has_current_joint_state_{false};

  // Geometricke konstanty robota
  const double l1_ = 0.0;
  const double l0_ = 0.0;
  const double l2_ = 0.203;
  const double l3_ = 0.203;
  const double l4_ = 0.05;
  const double d6_ = 0.15;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<IkSolverNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}