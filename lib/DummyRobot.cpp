//
// Created by markus on 24.04.21.
//

#include "DummyRobot.h"

DummyRobot::DummyRobot(std::string name, int radius, World *world) : Robot(world) {
    this->name = name;
    this->radius = radius;
}

DummyRobot::DummyRobot(std::string name, int radius, World *world, cv::Point2d start_pos, double start_orientation) : Robot(world) {
    this->name = name;
    this->radius = radius;
    this->pos = start_pos;
    this->orientation = start_orientation;
}

void DummyRobot::reposition(double x, double y, double angle) {
    this->pos = cv::Point2d(x,y);
    this->orientation = angle;
}
