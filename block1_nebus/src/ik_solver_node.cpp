#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter_client.hpp"

#include "sensor_msgs/msg/joint_state.hpp"

#include "block1_nebus/srv/get_ik_solutions.hpp"
#include "block1_nebus/srv/get_best_ik_solution.hpp"

#include <urdf/model.h>

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

class IkSolverNode : public rclcpp::Node
{
public:
  IkSolverNode() : Node("ik_solver_node")
  {
    // Subscriber na aktualny stav robota:
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&IkSolverNode::joint_states_callback, this, std::placeholders::_1)
    );

    // Service pre vypocet vsetkych validnych IK rieseni
    get_all_solutions_srv_ =
      this->create_service<block1_nebus::srv::GetIkSolutions>(
        "get_ik_solutions",
        std::bind(
          &IkSolverNode::handle_get_ik_solutions,
          this,
          std::placeholders::_1,
          std::placeholders::_2
        )
      );

    // Service pre vypocet najlepsieho IK riesenia
    get_best_solution_srv_ =
      this->create_service<block1_nebus::srv::GetBestIkSolution>(
        "get_best_ik_solution",
        std::bind(
          &IkSolverNode::handle_get_best_ik_solution,
          this,
          std::placeholders::_1,
          std::placeholders::_2
        )
      );

    // Pri starte sa pokusime nacitat joint limity z URDF.
    // SyncParametersClient je bezpecne pouzit pred startom executor spin-u.
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

    RCLCPP_INFO(this->get_logger(), "ik_solver_node started.");
  }

private:
  // Struktura pre joint limity nacitane z URDF
  struct JointLimits
  {
    double lower{0.0};
    double upper{0.0};
    bool valid{false};
  };

  // Jedno IK riesenie = jedna konfiguracia klbov
  struct JointSolution
  {
    double q1{0.0};
    double q2{0.0};
    double q3{0.0};
  };

  // =========================================================
  // 1. CALLBACK NA /joint_states
  // =========================================================
  //
  // Tu si node priebezne uklada aktualny stav robota.
  // Bez toho nevieme urcit "best solution".
  //
  void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (msg->name.empty() || msg->position.empty()) {
      return;
    }

    double q1 = 0.0;
    double q2 = 0.0;
    double q3 = 0.0;

    bool has_q1 = get_joint_value(*msg, "joint_1", q1);
    bool has_q2 = get_joint_value(*msg, "joint_2", q2);
    bool has_q3 = get_joint_value(*msg, "joint_3", q3);

    // Ak v sprave chyba niektory joint, nic neulozime
    if (!has_q1 || !has_q2 || !has_q3) {
      return;
    }

    // Ulozime si aktualne uhly, ale uz normalizovane do intervalu <-pi, pi>
    current_q1_ = normalize_angle(q1);
    current_q2_ = normalize_angle(q2);
    current_q3_ = normalize_angle(q3);
    has_current_joint_state_ = true;
  }

  // Pomocna funkcia:
  // zo spravy /joint_states vytiahne hodnotu konkretneho klbu podla mena
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

  // Normalizacia uhla do intervalu <-pi, pi>
  // Zadanie na to explicitne upozornuje. :contentReference[oaicite:2]{index=2}
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

  // Ochrana pred numerickou chybou pri acos / asin
  // Napr. ked vyjde 1.0000000001, co je matematicky takmer 1,
  // ale acos by na tom zlyhal.
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

  // =========================================================
  // 3. NACITANIE JOINT LIMITOV Z URDF
  // =========================================================
  //
  // Prednaska hovori:
  // - parameter robot_description je v inej node
  // - v ROS2 parametre nie su globalne
  // - preto pouzijeme SyncParametersClient
  // - ziskany URDF string naparsujeme cez urdf::Model
  //
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

    // Z robot_state_publisher si pytame parameter robot_description
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

    // Parsovanie URDF textu na model
    urdf::Model model;
    bool ok = model.initString(urdf_xml);

    if (!ok) {
      RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF from robot_description.");
      return false;
    }

    // Z URDF modelu vytiahneme limity pre joint_1, joint_2, joint_3
    bool ok1 = extract_joint_limits(model, "joint_1", joint1_limits_);
    bool ok2 = extract_joint_limits(model, "joint_2", joint2_limits_);
    bool ok3 = extract_joint_limits(model, "joint_3", joint3_limits_);

    if (!ok1 || !ok2 || !ok3) {
      RCLCPP_WARN(this->get_logger(), "Some joint limits could not be loaded from URDF.");
      return false;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Loaded joint limits: joint_1=[%.3f, %.3f], joint_2=[%.3f, %.3f], joint_3=[%.3f, %.3f]",
      joint1_limits_.lower, joint1_limits_.upper,
      joint2_limits_.lower, joint2_limits_.upper,
      joint3_limits_.lower, joint3_limits_.upper
    );

    has_joint_limits_ = true;
    return true;
  }

  // Z URDF modelu vytiahne lower / upper limity konkretneho jointu
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

  // Kontrola, ci jeden uhol patri do limitov jointu
  bool is_within_limits(double value, const JointLimits & limits) const
  {
    if (!limits.valid) {
      return false;
    }

    const double normalized = normalize_angle(value);
    return (normalized >= limits.lower && normalized <= limits.upper);
  }

  // Kontrola, ci cele jedno IK riesenie splna limity vsetkych troch jointov
  bool is_valid_solution(const JointSolution & solution) const
  {
    if (!has_joint_limits_) {
      return false;
    }

    return is_within_limits(solution.q1, joint1_limits_) &&
           is_within_limits(solution.q2, joint2_limits_) &&
           is_within_limits(solution.q3, joint3_limits_);
  }

  // =========================================================
  // 4. VYBER "BEST SOLUTION"
  // =========================================================
  //
  // Podla zadania je najlepsie riesenie to, ktore vyzaduje
  // najmensiu celkovu zmenu klbov vzhladom na aktualny stav robota.
  // Na to pouzijeme euklidovsku vzdialenost v priestore klbov.
  //
  double joint_distance_to_current(const JointSolution & solution) const
  {
    if (!has_current_joint_state_) {
      return std::numeric_limits<double>::infinity();
    }

    double dq1 = normalize_angle(solution.q1 - current_q1_);
    double dq2 = normalize_angle(solution.q2 - current_q2_);
    double dq3 = normalize_angle(solution.q3 - current_q3_);

    return std::sqrt(dq1 * dq1 + dq2 * dq2 + dq3 * dq3);
  }

  // =========================================================
  // 5. ANALYTICKY IK SOLVER - VSETKY KANDIDATNE RIESENIA
  // =========================================================
  //
  // Toto je najdolezitejsia cast node.
  //
  // Z tvojej priamej kinematiky vychadza, ze poloha TCP ma tvar:
  //
  // x = cos(q1) * ( l2*sin(q2) + l3*sin(q2 + q3) )
  // y = sin(q1) * ( l2*sin(q2) + l3*sin(q2 + q3) )
  // z = d + l2*cos(q2) + l3*cos(q2 + q3)
  //
  // kde:
  // d  = l0 + l1
  // l2 = dlzka druheho clanku
  // l3 = pevny offset od frame 3 po tool0
  //
  // Z toho spravime analyticky IK takto:
  // 1. z (x, y) vypocitame radialnu vzdialenost rho
  // 2. problem prevedieme na rovinu rho-z
  // 3. cez kosinovu vetu vypocitame q3
  // 4. potom dopocteme q2
  // 5. q1 ma dve moznosti, q3 ma dve moznosti -> spolu 4 kandidati
  //
  std::vector<JointSolution> compute_all_ik_solutions(double x, double y, double z)
  {
    std::vector<JointSolution> solutions;

    // Geometricke konstanty robota
    // d  = zvisly offset od zakladne po "rameno"
    // l2 = dlzka druheho clanku
    // l3 = posledny pevny offset od frame 3 po tool0
    const double d = l0_ + l1_;
    const double l2 = l2_;
    const double l3 = l3_;

    // Vzdialenost ciela od zvislej osi robota v rovine XY
    const double rho = std::sqrt(x * x + y * y);

    // Cielova vyska po odcitani zakladneho offsetu
    const double z_local = z - d;

    // Stvorcova vzdialenost ciela v rovine (rho, z_local)
    const double reach_sq = rho * rho + z_local * z_local;

    // Z kosinovej vety pre 2-clankovy mechanizmus dostaneme cos(q3)
    const double raw_cos_q3 = (reach_sq - l2 * l2 - l3 * l3) / (2.0 * l2 * l3);

    // Ak je ciel mimo pracovneho priestoru, realne riesenie neexistuje
    const double eps = 1e-9;
    if (raw_cos_q3 < -1.0 - eps || raw_cos_q3 > 1.0 + eps) {
      return solutions;
    }

    // Ochrana pred numerickymi chybami pri volani acos
    const double cos_q3 = clamp_to_unit_interval(raw_cos_q3);

    // Pre q3 dostaneme 2 moznosti:
    // - elbow up
    // - elbow down
    const double q3_a = std::acos(cos_q3);
    const double q3_b = -std::acos(cos_q3);

    // Zakladne riesenie pre q1 je smer na bod v rovine XY
    const double q1_base = std::atan2(y, x);

    // Pre q1 existuju 2 moznosti:
    // 1. priamo smerom k bodu
    // 2. otocenie o pi (robot sa "pozrie" opacne)
    const double q1_candidates[2] = {
      q1_base,
      q1_base + M_PI
    };

    // Ked q1 posunieme o pi, musime aj radialnu vzdialenost brat so zapornym znamienkom,
    // aby rovnice ostali konzistentne
    const double rho_candidates[2] = {
      rho,
      -rho
    };

    // 2 moznosti pre q3
    const double q3_candidates[2] = {
      q3_a,
      q3_b
    };

    // Kombinaciou 2 moznosti pre q1 a 2 moznosti pre q3
    // dostaneme maximalne 4 kandidatne riesenia
    for (int i = 0; i < 2; ++i) {
      const double q1 = q1_candidates[i];
      const double rho_signed = rho_candidates[i];

      for (int j = 0; j < 2; ++j) {
        const double q3 = q3_candidates[j];

        // Pomocne vyrazy pre vypocet q2
        const double k1 = l2 + l3 * std::cos(q3);
        const double k2 = l3 * std::sin(q3);

        // q2 vypocitame analyticky:
        // rho_signed = l2*sin(q2) + l3*sin(q2 + q3)
        // z_local    = l2*cos(q2) + l3*cos(q2 + q3)
        const double q2 =
          std::atan2(rho_signed, z_local) - std::atan2(k2, k1);

        JointSolution sol;
        sol.q1 = normalize_angle(q1);
        sol.q2 = normalize_angle(q2);
        sol.q3 = normalize_angle(q3);

        // Pri singularitach sa moze stat, ze dve "teoreticky rozne" vetvy daju
        // rovnake riesenie. Vtedy nechceme pridavat duplicitu.
        bool duplicate = false;
        for (const auto & existing : solutions) {
          const double dq1 = std::abs(normalize_angle(sol.q1 - existing.q1));
          const double dq2 = std::abs(normalize_angle(sol.q2 - existing.q2));
          const double dq3 = std::abs(normalize_angle(sol.q3 - existing.q3));

          if (dq1 < 1e-6 && dq2 < 1e-6 && dq3 < 1e-6) {
            duplicate = true;
            break;
          }
        }

        if (!duplicate) {
          solutions.push_back(sol);
        }
      }
    }

    return solutions;
  }

  // Z kandidatnych rieseni vyberie len tie, ktore splnaju joint limity
  std::vector<JointSolution> compute_valid_ik_solutions(double x, double y, double z)
  {
    std::vector<JointSolution> all_solutions = compute_all_ik_solutions(x, y, z);
    std::vector<JointSolution> valid_solutions;

    for (const auto & sol : all_solutions) {
      JointSolution normalized_sol;
      normalized_sol.q1 = normalize_angle(sol.q1);
      normalized_sol.q2 = normalize_angle(sol.q2);
      normalized_sol.q3 = normalize_angle(sol.q3);

      if (is_valid_solution(normalized_sol)) {
        valid_solutions.push_back(normalized_sol);
      }
    }

    return valid_solutions;
  }

  // =========================================================
  // 6. SERVICE: VSETKY VALIDNE RIESENIA
  // =========================================================
  //
  // Vstup: cielovy bod x, y, z
  // Vystup: vsetky validne riesenia, ktore splnaju joint limity
  //
  void handle_get_ik_solutions(
    const std::shared_ptr<block1_nebus::srv::GetIkSolutions::Request> request,
    std::shared_ptr<block1_nebus::srv::GetIkSolutions::Response> response)
  {
    if (!has_joint_limits_) {
      response->success = false;
      response->message = "Joint limits are not available.";
      response->solution_count = 0;
      return;
    }

    // Vypocitame iba validne riesenia
    std::vector<JointSolution> valid_solutions =
      compute_valid_ik_solutions(request->x, request->y, request->z);

    response->solution_count = static_cast<uint32_t>(valid_solutions.size());

    if (valid_solutions.empty()) {
      response->success = false;
      response->message = "No valid IK solution found.";
      return;
    }

    // Do response zapiseme vsetky najdene riesenia
    for (const auto & sol : valid_solutions) {
      response->q1.push_back(sol.q1);
      response->q2.push_back(sol.q2);
      response->q3.push_back(sol.q3);
    }

    response->success = true;
    response->message = "Valid IK solutions found.";
  }

  // =========================================================
  // 7. SERVICE: NAJLEPSIE RIESENIE
  // =========================================================
  //
  // Vstup: cielovy bod x, y, z
  // Vystup: jedno najlepsie riesenie = najblizsie k aktualnemu stavu robota
  //
  void handle_get_best_ik_solution(
    const std::shared_ptr<block1_nebus::srv::GetBestIkSolution::Request> request,
    std::shared_ptr<block1_nebus::srv::GetBestIkSolution::Response> response)
  {
    if (!has_joint_limits_) {
      response->success = false;
      response->message = "Joint limits are not available.";
      return;
    }

    // Na best solution potrebujeme poznat aktualny stav robota
    if (!has_current_joint_state_) {
      response->success = false;
      response->message = "Current joint state is not available yet.";
      return;
    }

    // Vypocitame iba validne riesenia
    std::vector<JointSolution> valid_solutions =
      compute_valid_ik_solutions(request->x, request->y, request->z);

    if (valid_solutions.empty()) {
      response->success = false;
      response->message = "No valid IK solution found.";
      return;
    }

    // Najdeme take riesenie, ktore ma najmensiu joint-space vzdialenost
    // od aktualnej konfiguracie robota
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
    response->success = true;
    response->message = "Best IK solution found.";
  }

  // =========================================================
  // 8. CLENSKE PREMENNE
  // =========================================================

  // Subscriber
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  // Services
  rclcpp::Service<block1_nebus::srv::GetIkSolutions>::SharedPtr get_all_solutions_srv_;
  rclcpp::Service<block1_nebus::srv::GetBestIkSolution>::SharedPtr get_best_solution_srv_;

  // Joint limity z URDF
  JointLimits joint1_limits_;
  JointLimits joint2_limits_;
  JointLimits joint3_limits_;
  bool has_joint_limits_{false};

  // Aktualny stav robota z /joint_states
  double current_q1_{0.0};
  double current_q2_{0.0};
  double current_q3_{0.0};
  bool has_current_joint_state_{false};

  // Geometricke konstanty robota
  // Musia sediet s forward kinematics v logger_node.cpp
  const double l1_ = 0.0;
  const double l0_ = 0.0;
  const double l2_ = 0.203;
  const double l3_ = 0.203;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<IkSolverNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}