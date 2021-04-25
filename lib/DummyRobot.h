//
// Created by markus on 24.04.21.
//

#ifndef MR_CPP_CODE_DUMMYROBOT_H
#define MR_CPP_CODE_DUMMYROBOT_H


#include "../Simulator/interactive_objects/robot.h"

class DummyRobot : public Robot {
public:
    DummyRobot(std::string name, int radius, World* world);

    DummyRobot(std::string name, int radius, World* world, cv::Point2d start_pos, double start_orientation);

    void handleCollision(CollidableObject *object) override {};

    void update() {};

    void reposition(double x, double y, double angle);
};


#endif //MR_CPP_CODE_DUMMYROBOT_H
