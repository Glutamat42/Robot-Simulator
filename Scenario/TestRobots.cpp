//
// Created by markus on 19.04.21.
//

#include <unistd.h>
#include "TestRobots.h"
#include "../Operators/BasicWithNovelty.h"

void TestRobots::setUp() {
    this->world = new World("assets/maps/world1.png");

    // robot 1
    Robot r = Robot(std::string("r1"), 8, cv::Point2d(320.0, 40.0), 1.18 * M_PI, world, M_PI / 3, 10);
    DistanceSensor s1 = DistanceSensor(world, &r, 0.20 * M_PI, 200);
    DistanceSensor s2 = DistanceSensor(world, &r, -0.20 * M_PI, 200);
    DistanceSensor s3 = DistanceSensor(world, &r, 0.45 * M_PI, 200);
    DistanceSensor s4 = DistanceSensor(world, &r, -0.45 * M_PI, 200);
    r.add_sensor(&s1);
    r.add_sensor(&s2);
    r.add_sensor(&s3);
    r.add_sensor(&s4);
    world->add_object(&r);
    RobotOperator* ro = new BasicWithNovelty(&r);

    // robot 2
    Robot r2 = Robot(std::string("r2"), 8, cv::Point2d(280.0, 50.0), 0.15 * M_PI, world, M_PI / 3, 10);
    DistanceSensor r2s1 = DistanceSensor(world, &r2, 0.2*M_PI, 200);
    r2.add_sensor(&r2s1);
    DistanceSensor r2s2 = DistanceSensor(world, &r2, -0.2*M_PI, 200);
    r2.add_sensor(&r2s2);
    world->add_object(&r2);
    RobotOperator* ro2 = new BasicWithNovelty(&r2);


    sensors.push_back(&s1);
    sensors.push_back(&s2);
    sensors.push_back(&s3);
    sensors.push_back(&s4);
    sensors.push_back(&r2s1);
    sensors.push_back(&r2s2);
    robots.push_back(&r);
    robots.push_back(&r2);
    robotOperators.push_back(ro);
    robotOperators.push_back(ro2);


    world->show_map();
    cv::waitKey();
    r.update();
    r2.update();
    world->show_map();
    cv::waitKey();
    sleep(5);
    cv::waitKey();
}
