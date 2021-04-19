#include <iostream>
#include "Simulator/world.h"
#include "Simulator/interactive_objects/robot.h"
#include "Simulator/interactive_objects/DistanceSensor.h"
#include "Simulator/constants.h"
#include "Simulator/RobotOperator.h"
#include "Scenario/RobotArmy.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <unistd.h>

using chrono_clock = std::chrono::system_clock;
using chrono_ms = std::chrono::duration<double, std::milli>;


int main() {
    srand(time(NULL));

//    SimulationScenario* scenario = new TestRobots();
    SimulationScenario* scenario = new RobotArmy();
    scenario->setUp();
    scenario->startLoop();







    return 0;
}
