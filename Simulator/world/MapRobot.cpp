//
// Created by markus on 24.04.21.
//

#include "MapRobot.h"

MapRobot::MapRobot(double radius) {
    this->radius = radius;
}

MapRobot::MapRobot(double radius, double angle, cv::Point2d pos) {
    this->radius = radius;
    this->angle = angle;
    this->pos = pos;
}

void MapRobot::draw(cv::Mat image) {
    cv::circle(image, this->pos, radius, robotCircleColor, 1);
    line(image,
         this->pos,
         this->pos + cv::Point2d(cos(this->angle) * this->radius,
                                            sin(this->angle) * this->radius),
         CV_RGB(0, 255, 0),
         1);
}

void MapRobot::setColor(cv::Scalar color) {
    this->robotCircleColor = color;
}

void MapRobot::reposition(cv::Point2d pos, double angle) {
    this->pos = pos;
    this->angle = angle;
}
