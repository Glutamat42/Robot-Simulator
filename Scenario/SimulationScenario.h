//
// Created by markus on 19.04.21.
//

#ifndef MR_CPP_CODE_SIMULATIONSCENARIO_H
#define MR_CPP_CODE_SIMULATIONSCENARIO_H


#include <vector>
#include "../Simulator/interactive_objects/robot.h"
#include "../Operators/RobotOperator.h"
#include "../Simulator/interactive_objects/DistanceSensor.h"

class SimulationScenario {
protected:
    World *world;
    std::vector<Robot *> robots;
    std::vector<RobotOperator *> robotOperators;
    std::vector<DistanceSensor *> sensors;
public:
    virtual ~SimulationScenario();

    void init(bool pause = false);

    virtual void setUp() = 0;

    void startLoop();
};


#endif //MR_CPP_CODE_SIMULATIONSCENARIO_H
