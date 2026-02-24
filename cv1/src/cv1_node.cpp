#include "rclcpp/rclcpp.hpp"
#include "cv1/robot.hpp"
#include <iostream>
#include <string>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv); //initialize ros2

    Robot robot; //creates robot

    RCLCPP_INFO(rclcpp::get_logger("cv1"),
                "Ovládanie: w/a/s/d + Enter, ukonči: q + Enter");

    while (rclcpp::ok()) {
        std::cout << "Zadaj prikaz (w/a/s/d/q): ";
        std::string cmd;
        std::cin >> cmd;

        char c = cmd[0]; //we take only the first character

        double dx = 0.0;
        double dy = 0.0;

        if (c == 'w') { dy =  1.0; }
        else if (c == 's') { dy = -1.0; }
        else if (c == 'a') { dx = -1.0; }
        else if (c == 'd') { dx =  1.0; }
        else if (c == 'q') { 
            RCLCPP_INFO(rclcpp::get_logger("cv1"), "Koniec programu.");
            break;
        }
        else {
            std::cout << "Neznamy prikaz. Pouzi w/a/s/d alebo q.\n";
            continue;
        }

        robot.move(dx, dy); //move the robot (add dx to x and dy to y)

        RCLCPP_INFO(rclcpp::get_logger("cv1"),
                    "Aktualna poloha: x=%.2f, y=%.2f",
                    robot.getX(), robot.getY());
    }

    rclcpp::shutdown();
    return 0;
}
