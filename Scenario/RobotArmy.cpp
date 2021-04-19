//
// Created by markus on 19.04.21.
//

#include "RobotArmy.h"

void RobotArmy::setUp() {
    this->world = new World("assets/maps/world1.png");

    for (int i = 0; i < 50; ++i) {
        Robot* robot = new Robot(std::string("robot_army_" + std::to_string(i)), 8, world, M_PI / 3, 200);
        world->add_object(robot);

        DistanceSensor* sensor1 = new DistanceSensor(world, robot, 0.2*M_PI, 200);
        DistanceSensor* sensor2 = new DistanceSensor(world, robot, -0.2*M_PI, 200);
        robot->add_sensor(sensor1);
        robot->add_sensor(sensor2);

        RobotOperator* robotOperator = new RobotOperator(robot);

        sensors.push_back(sensor1);
        sensors.push_back(sensor2);
        robots.push_back(robot);
        robotOperators.push_back(robotOperator);
    }

    world->show_map();
    cv::waitKey();
}
