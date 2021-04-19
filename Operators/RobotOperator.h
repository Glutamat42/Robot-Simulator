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

    std::vector<DistanceSensor*> filter_for_distance_sensor(std::vector<SensorInterface*> sensors);
public:
    RobotOperator(RobotControlInterface* robot);

    virtual void update() = 0;
};


#endif //MR_CPP_CODE_ROBOTOPERATOR_H
