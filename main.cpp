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
#include "Scenario/SelfDrivingScenario.h"
#include "lib/PathFindingAlgorithm/PathFindingAlgorithm.h"
#include "lib/ValueIteration/ValueIteration.h"
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
//    SimulationScenario* scenario = new AStarScenario();
//    SimulationScenario* scenario = new SelfDrivingScenario();
//    scenario->setUp();
//    scenario->startLoop();


    ValueIteration* valueIteration = new ValueIteration(std::string("assets/maps/world1.png"), 0, 40);
    valueIteration->setParams(cv::Point2i(400,400), cv::Point2i(600,700), 1, cv::Point2i(150,700), -1, -0.04, 0.9);
    PathFindingAlgorithm<ValueIterationElement>* pathFindingAlgorithm = valueIteration;
    pathFindingAlgorithm->run();







    return 0;
}
