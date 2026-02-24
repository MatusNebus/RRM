#include "cv1/robot.hpp"
#include "rclcpp/rclcpp.hpp"

Robot::Robot() : x_(0.0), y_(0.0) {
    RCLCPP_INFO(rclcpp::get_logger("cv1"), "Ahoj, ja som robot. Začiatočná pozícia: (%.2f, %.2f)", x_, y_);
}

void Robot::move(double dx, double dy) {
    x_ += dx;
    y_ += dy;
}

double Robot::getX() const {
    return x_;
}

double Robot::getY() const {
    return y_;
}
