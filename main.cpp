#include <iostream>
#include "Simulator/world/world.h"
#include "Simulator/interactive_objects/robot.h"
#include "Simulator/interactive_objects/DistanceSensor.h"
#include "Simulator/constants.h"
#include "Operators/RobotOperator.h"
#include "Scenario/RobotArmy.h"
#include "Scenario/ParticleScenario.h"
#include "Scenario/TestRobots.h"
#include "Scenario/AStarScenario.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <unistd.h>

using chrono_clock = std::chrono::system_clock;
using chrono_ms = std::chrono::duration<double, std::milli>;


int main() {
    srand(time(NULL));

//    SimulationScenario* scenario = new TestRobots();
//    SimulationScenario* scenario = new RobotArmy();
//    SimulationScenario* scenario = new ParticleScenario();
    SimulationScenario* scenario = new AStarScenario();
    scenario->setUp();
    scenario->startLoop();







    return 0;
}
