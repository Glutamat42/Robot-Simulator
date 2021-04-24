//
// Created by markus on 22.04.21.
//

#include "AStarOperator.h"
#include "../lib/helpers.h"
#include "../lib/AStar.h"

AStarOperator::AStarOperator(RobotControlInterface *robot, std::string map_filename) : RobotOperator(robot) {
    cv::Point2i startLocation(310, 300);
    cv::Point2i targetLocation(750, 700);
    AStar aStar = AStar(map_filename, this->robot->get_radius() + 3);;
    aStar.setAStarParameters(startLocation, targetLocation, 1.4);
    std::vector<cv::Point2d> path = AStar::aStarListToPointList(aStar.runAStar());

//    exit(0);
    cv::waitKey(0);
    std::cout << "HELLO WORLD" << std::endl;
}

void AStarOperator::update() {

}
