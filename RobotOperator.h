//
// Created by markus on 05.04.21.
//

#ifndef MR_CPP_CODE_ROBOTOPERATOR_H
#define MR_CPP_CODE_ROBOTOPERATOR_H


#include "robot.h"
#include "DistanceSensor.h"
#include "interfaces/RobotControlInterface.h"
#include "constants.h"

class RobotOperator {
private:
    RobotControlInterface* robot;
    std::vector<DistanceSensor*> filter_for_distance_sensor(std::vector<SensorInterface*> sensors);
    int distance_sensor_history_index = 0;
    int distance_sensor_history_size = (int) (100 * TARGET_TPS);
    std::vector<std::vector<int>> distance_sensor_history;
    int novelty_score = 0;
    int novelty_weight = 0;
    bool unstucking = false;
public:
    RobotOperator(RobotControlInterface* robot);

    void update();

    bool novelty_detection();
};


#endif //MR_CPP_CODE_ROBOTOPERATOR_H
