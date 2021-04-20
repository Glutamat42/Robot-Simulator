//
// Created by markus on 20.04.21.
//

#include "ParticleScenario.h"
#include "../Operators/ParticleOperator.h"

void ParticleScenario::setUp() {
    std::string mapName = "assets/maps/world1.png";
    this->world = new World(mapName);

    // robot 1
    Robot* r = new Robot(std::string("r1"), 8, cv::Point2d(320.0, 40.0), 1.18 * M_PI, world, M_PI / 2, 40);
    DistanceSensor* s1 = new DistanceSensor(world, r, 0.15*M_PI, 200);
    DistanceSensor* s3 = new DistanceSensor(world, r, 0.25*M_PI, 200);
    DistanceSensor* s2 = new DistanceSensor(world, r, -0.15*M_PI, 200);
    DistanceSensor* s4 = new DistanceSensor(world, r, -0.25*M_PI, 200);
    r->add_sensor(s1);
    r->add_sensor(s2);
    r->add_sensor(s3);
    r->add_sensor(s4);
    world->add_object(r);
    RobotOperator* ro = new ParticleOperator(r, mapName);

    sensors.push_back(s1);
    sensors.push_back(s2);
    sensors.push_back(s3);
    sensors.push_back(s4);
    robots.push_back(r);
    robotOperators.push_back(ro);


    world->show_map();
    cv::waitKey();
}
