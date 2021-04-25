//
// Created by markus on 22.04.21.
//

#include <chrono>
#include "AStarOperator.h"
#include "../lib/helpers.h"
#include "../lib/FastAStar.h"

using chrono_clock = std::chrono::system_clock;
using chrono_ms = std::chrono::duration<double, std::milli>;

AStarOperator::AStarOperator(RobotControlInterface *robot, std::string map_filename) : RobotOperator(robot) {
    cv::Point2i startLocation(310, 300);
    cv::Point2i targetLocation(750, 700);

//    AStar aStar = AStar(map_filename, this->robot->get_radius() + 3);;
    FastAStar aStar = FastAStar(map_filename, this->robot->get_radius() + 3);;

//    std::vector<cv::Point2d> path = AStar::aStarListToPointList(aStar.runAStar());
    const auto loop_start_chrono = chrono_clock::now();
//    aStar.setAStarParameters(startLocation, targetLocation, 1.0);
//    aStar.runAStar();
    aStar.setAStarParameters(cv::Point2i(40, 40), targetLocation, 1.0);
    aStar.runAStar();
    const chrono_ms loop_duration = chrono_clock::now() - loop_start_chrono;
    std::cout << loop_duration.count() << "ms";


//    exit(0);
    cv::waitKey(0);
    std::cout << "HELLO WORLD" << std::endl;
}

void AStarOperator::update() {

}
