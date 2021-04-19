//
// Created by markus on 20.04.21.
//

#ifndef MR_CPP_CODE_BASICWITHNOVELTY_H
#define MR_CPP_CODE_BASICWITHNOVELTY_H

#include "../Simulator/interactive_objects/robot.h"
#include "../Simulator/interactive_objects/DistanceSensor.h"
#include "../Simulator/interactive_objects/interfaces/RobotControlInterface.h"
#include "../Simulator/constants.h"
#include "RobotOperator.h"

class BasicWithNovelty : public RobotOperator {
protected:
    int distance_sensor_history_index = 0;
    int distance_sensor_history_size = (int) (100 * GAME_TPS);
    std::vector<std::vector<int>> distance_sensor_history;
    int novelty_score = 0;
    int novelty_weight = 0;
    bool unstucking = false;
public:
    BasicWithNovelty(RobotControlInterface* robot);

    void update() override;

    bool novelty_detection();
};


#endif //MR_CPP_CODE_BASICWITHNOVELTY_H
