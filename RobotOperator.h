//
// Created by markus on 05.04.21.
//

#ifndef MR_CPP_CODE_ROBOTOPERATOR_H
#define MR_CPP_CODE_ROBOTOPERATOR_H


#include "robot.h"
#include "sensor.h"

class RobotOperator {
private:
    Robot* robot;
public:
    RobotOperator(Robot* robot);

    void update();
};


#endif //MR_CPP_CODE_ROBOTOPERATOR_H
