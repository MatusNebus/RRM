#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"

#include "sensor_msgs/msg/joint_state.hpp"

#include "rrm_msgs/srv/command.hpp"
#include "block1_nebus/srv/move_to_point.hpp"
#include "block1_nebus/srv/get_best_ik_solution.hpp"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

class MotionManagerNode : public rclcpp::Node
{
public:
  MotionManagerNode() : Node("motion_manager_node")
  {
    service_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    state_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // Parameter:
    // maximalna rychlost, ktoru smie manager poslat do move_command.
    // Pouzijeme ju na synchronizaciu pohybu vsetkych jointov.
    this->declare_parameter("max_velocity", 0.5);
    this->get_parameter("max_velocity", max_velocity_);

    // Subscriber na aktualny stav robota.
    // Zadanie hovori, ze manager ma ziskat aktualny stav robota.
    // Tu si ho budeme priebezne pamatat.
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = state_cb_group_;

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&MotionManagerNode::joint_states_callback, this, std::placeholders::_1),
      sub_options
    );

    // Klient na IK solver:
    // manager sa bude pytat na "najlepsie" IK riesenie
    best_ik_client_ =
      this->create_client<block1_nebus::srv::GetBestIkSolution>(
        "get_best_ik_solution",
        rmw_qos_profile_services_default,
        client_cb_group_);

    // Klient na simulaciu:
    // sem posleme joint uhly a rychlosti
    move_command_client_ =
      this->create_client<rrm_msgs::srv::Command>(
        "move_command",
        rmw_qos_profile_services_default,
        client_cb_group_);

    // Service pre pouzivatela:
    // vstup = cielovy bod x, y, z
    // manager sam vybavi zvysok
    move_to_point_srv_ =
      this->create_service<block1_nebus::srv::MoveToPoint>(
        "move_to_point",
        std::bind(
          &MotionManagerNode::handle_move_to_point,
          this,
          std::placeholders::_1,
          std::placeholders::_2
        ),
        rmw_qos_profile_services_default,
        service_cb_group_
      );

    RCLCPP_INFO(this->get_logger(), "motion_manager_node started.");
  }

private:
  // =========================================================
  // 1. CALLBACK NA /joint_states
  // =========================================================
  //
  // Ukladame si aktualne uhly joint_1, joint_2, joint_3.
  // Potrebujeme ich:
  // - aby sme vedeli, ci uz robot realne publikuje stav
  // - aby sme vedeli vypocitat rozumne rychlosti pre move_command
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

    if (!has_q1 || !has_q2 || !has_q3) {
      return;
    }

    current_q1_ = normalize_angle(q1);
    current_q2_ = normalize_angle(q2);
    current_q3_ = normalize_angle(q3);
    has_current_joint_state_ = true;
  }

  // Pomocna funkcia:
  // zo spravy /joint_states vytiahne hodnotu jointu podla mena
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

  // Normalizacia uhla do intervalu <-pi, pi>
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

  // =========================================================
  // 2. VYPOCET RYCHLOSTI PRE move_command
  // =========================================================
  //
  // Service move_command chce:
  // - positions[]
  // - velocities[]
  //
  // Chceme jednoduchy a pekny pohyb:
  // joint s najvacsou zmenou pojde rychlostou max_velocity_
  // ostatne jointy dostanu pomerne mensie rychlosti,
  // aby vsetky dosli priblizne naraz.
  //
  std::vector<double> compute_synchronized_velocities(
    double target_q1, double target_q2, double target_q3) const
  {
    const double dq1 = std::abs(normalize_angle(target_q1 - current_q1_));
    const double dq2 = std::abs(normalize_angle(target_q2 - current_q2_));
    const double dq3 = std::abs(normalize_angle(target_q3 - current_q3_));

    const double max_delta = std::max({dq1, dq2, dq3});
    const double min_velocity = 0.05;  // mala nenulova rychlost, aby joint "nestal"

    std::vector<double> velocities(3, min_velocity);

    // Ak je ciel prakticky rovnaky ako aktualna poloha,
    // vratime male rychlosti. Simulacia by to mala vybavit velmi rychlo.
    if (max_delta < 1e-9) {
      return velocities;
    }

    velocities[0] = std::max(min_velocity, max_velocity_ * (dq1 / max_delta));
    velocities[1] = std::max(min_velocity, max_velocity_ * (dq2 / max_delta));
    velocities[2] = std::max(min_velocity, max_velocity_ * (dq3 / max_delta));

    return velocities;
  }

  // =========================================================
  // 3. SERVICE HANDLER PRE move_to_point
  // =========================================================
  //
  // Toto je hlavna logika celeho motion managera.
  // Ked pouzivatel zavola service move_to_point:
  // 1. overime, ze mame aktualny stav robota
  // 2. popytame sa IK solvera na najlepsie riesenie
  // 3. ziskane joint uhly posleme do simulacie cez move_command
  // 4. az ked move_command odpovie, vratime success / error
  //
  void handle_move_to_point(
    const std::shared_ptr<block1_nebus::srv::MoveToPoint::Request> request,
    std::shared_ptr<block1_nebus::srv::MoveToPoint::Response> response)
  {
    // Zadanie hovori, ze manager ma pracovat s aktualnym stavom robota.
    // Preto bez /joint_states radsej nepokracujeme.
    if (!has_current_joint_state_) {
      response->success = false;
      response->message = "Current joint state is not available yet.";
      return;
    }

    // Pockame, kym bude dostupny IK solver
    if (!best_ik_client_->wait_for_service(std::chrono::seconds(3))) {
      response->success = false;
      response->message = "Service get_best_ik_solution is not available.";
      return;
    }

    // Zlozime request na IK solver
    auto ik_request =
      std::make_shared<block1_nebus::srv::GetBestIkSolution::Request>();
    ik_request->x = request->x;
    ik_request->y = request->y;
    ik_request->z = request->z;

    // Zavolame IK solver
    auto ik_future = best_ik_client_->async_send_request(ik_request);

    auto ik_status = ik_future.wait_for(std::chrono::seconds(5));
    if (ik_status != std::future_status::ready) {
      response->success = false;
      response->message = "Timeout while waiting for IK solver response.";
      return;
    }

    auto ik_result = ik_future.get();

    if (!ik_result->success) {
      response->success = false;
      response->message = "IK solver failed: " + ik_result->message;
      return;
    }

    // Mame najlepsie IK riesenie
    const double target_q1 = normalize_angle(ik_result->q1);
    const double target_q2 = normalize_angle(ik_result->q2);
    const double target_q3 = normalize_angle(ik_result->q3);

    // Pripravime rychlosti tak, aby vsetky jointy prisli priblizne naraz
    std::vector<double> velocities =
      compute_synchronized_velocities(target_q1, target_q2, target_q3);

    // Pockame, kym bude dostupna service move_command
    if (!move_command_client_->wait_for_service(std::chrono::seconds(3))) {
      response->success = false;
      response->message = "Service move_command is not available.";
      return;
    }

    // Zlozime request pre simulaciu
    auto move_request = std::make_shared<rrm_msgs::srv::Command::Request>();
    move_request->positions = {target_q1, target_q2, target_q3};
    move_request->velocities = velocities;

    // Posleme prikaz na pohyb
    auto move_future = move_command_client_->async_send_request(move_request);

    // success vratime az ked simulacia potvrdi ukoncenie pohybu.
    auto move_status = move_future.wait_for(std::chrono::seconds(60)); //pohyb moze trvat najviac minutu
    if (move_status != std::future_status::ready) {
      response->success = false;
      response->message = "Timeout while waiting for move_command response.";
      return;
    }

    auto move_result = move_future.get();

    // Predpoklad:
    // result_code == 0 znamena uspech
    // ak mate v simulatore inu konvenciu, treba to pripadne zmenit
    if (move_result->result_code == 0) {
      response->success = true;
      response->message = "Motion completed successfully: " + move_result->message;
    } else {
      response->success = false;
      response->message =
        "Motion failed, result_code=" + std::to_string(move_result->result_code) +
        ", message=" + move_result->message;
    }
  }

  // =========================================================
  // 4. CLENSKE PREMENNE
  // =========================================================

  // Subscriber
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  // Callback groups to allow service callback to block while client responses are processed
  rclcpp::CallbackGroup::SharedPtr service_cb_group_;
  rclcpp::CallbackGroup::SharedPtr client_cb_group_;
  rclcpp::CallbackGroup::SharedPtr state_cb_group_;

  // Klienti na ine services
  rclcpp::Client<block1_nebus::srv::GetBestIkSolution>::SharedPtr best_ik_client_;
  rclcpp::Client<rrm_msgs::srv::Command>::SharedPtr move_command_client_;

  // Service pre pouzivatela
  rclcpp::Service<block1_nebus::srv::MoveToPoint>::SharedPtr move_to_point_srv_;

  // Aktualny stav robota
  double current_q1_{0.0};
  double current_q2_{0.0};
  double current_q3_{0.0};
  bool has_current_joint_state_{false};

  // Parameter
  double max_velocity_{1.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MotionManagerNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}