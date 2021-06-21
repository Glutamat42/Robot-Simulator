//
// Created by markus on 05.04.21.
//

#ifndef MR_CPP_CODE_ROBOTOPERATOR_H
#define MR_CPP_CODE_ROBOTOPERATOR_H


#include "../Simulator/interactive_objects/robot.h"
#include "../Simulator/interactive_objects/DistanceSensor.h"
#include "../Simulator/interactive_objects/interfaces/RobotControlInterface.h"
#include "../Simulator/constants.h"

class RobotOperator {
protected:
    RobotControlInterface* robot;
public:
    explicit RobotOperator(RobotControlInterface* robot);

    virtual ~RobotOperator() = default;

    virtual void afterUpdate() {};

    virtual void update() = 0;
};


#endif //MR_CPP_CODE_ROBOTOPERATOR_H
