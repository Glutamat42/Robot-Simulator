//
// Created by markus on 20.04.21.
//

#include "ParticleScenario.h"
#include "../Operators/ParticleOperator.h"

void ParticleScenario::setUp() {

    bool BENCHMARK_MODE = true;


    std::string mapName = "assets/maps/world1.png";
    this->world = new World(mapName);

    if (BENCHMARK_MODE) {
        std::cout << "Benchmark mode enabled" << std::endl
                  << "IMPORTANT: To get comparable values the constants (eg tps) have to be the same. They are not automatically by enabling the benchmark mode!"
                  << std::endl;

        Robot *r = new Robot(std::string("r1"), 8, cv::Point2d(310.0, 300.0), 1.0 * M_PI, world, M_PI / 2, 40);
        DistanceSensor *s1 = new DistanceSensor(world, r, 0.15 * M_PI, 800);
        DistanceSensor *s2 = new DistanceSensor(world, r, 0.25 * M_PI, 800);
        DistanceSensor *s3 = new DistanceSensor(world, r, -0.15 * M_PI, 800);
        DistanceSensor *s4 = new DistanceSensor(world, r, -0.25 * M_PI, 800);
        DistanceSensor *s5 = new DistanceSensor(world, r, 0.05 * M_PI, 800);
        DistanceSensor *s6 = new DistanceSensor(world, r, -0.05 * M_PI, 800);
        DistanceSensor *s7 = new DistanceSensor(world, r, -0.35 * M_PI, 800);
        DistanceSensor *s8 = new DistanceSensor(world, r, -0.35 * M_PI, 800);
        r->add_sensor(s1);
        r->add_sensor(s2);
        r->add_sensor(s3);
        r->add_sensor(s4);
        r->add_sensor(s5);
        r->add_sensor(s6);
        r->add_sensor(s7);
        r->add_sensor(s8);

        world->add_object(r);
        RobotOperator *ro = new ParticleOperator(r, mapName, true);

        sensors.push_back(s1);
        sensors.push_back(s2);
        sensors.push_back(s3);
        sensors.push_back(s4);
        sensors.push_back(s5);
        sensors.push_back(s6);
        sensors.push_back(s7);
        sensors.push_back(s8);
        robots.push_back(r);
        robotOperators.push_back(ro);

        this->init(false);
    } else {
        // robot 1
//    Robot* r = new Robot(std::string("r1"), 8, cv::Point2d(320.0, 40.0), 1.18 * M_PI, world, M_PI / 2, 40);
//    Robot* r = new Robot(std::string("r1"), 8, cv::Point2d(310.0, 300.0), 1.0 * M_PI, world, M_PI / 2, 40);
        Robot *r = new Robot(std::string("r1"), 8, world, M_PI / 2, 40);
        DistanceSensor *s1 = new DistanceSensor(world, r, 0.15 * M_PI, 800);
        DistanceSensor *s2 = new DistanceSensor(world, r, 0.25 * M_PI, 800);
        DistanceSensor *s3 = new DistanceSensor(world, r, -0.15 * M_PI, 800);
        DistanceSensor *s4 = new DistanceSensor(world, r, -0.25 * M_PI, 800);
        DistanceSensor *s5 = new DistanceSensor(world, r, 0.05 * M_PI, 800);
        DistanceSensor *s6 = new DistanceSensor(world, r, -0.05 * M_PI, 800);
        DistanceSensor *s7 = new DistanceSensor(world, r, -0.35 * M_PI, 800);
        DistanceSensor *s8 = new DistanceSensor(world, r, -0.35 * M_PI, 800);
        r->add_sensor(s1);
        r->add_sensor(s2);
        r->add_sensor(s3);
        r->add_sensor(s4);
        r->add_sensor(s5);
        r->add_sensor(s6);
        r->add_sensor(s7);
        r->add_sensor(s8);
//     "Lidar"
//    for (double i = -M_PI; i < M_PI; i+=0.4) {
//        DistanceSensor* sens = new DistanceSensor(world, r, i, 800);
//        r->add_sensor(sens);
//        sensors.push_back(sens);
//    }
        world->add_object(r);
        RobotOperator *ro = new ParticleOperator(r, mapName);

        sensors.push_back(s1);
        sensors.push_back(s2);
        sensors.push_back(s3);
        sensors.push_back(s4);
        sensors.push_back(s5);
        sensors.push_back(s6);
        sensors.push_back(s7);
        sensors.push_back(s8);
        robots.push_back(r);
        robotOperators.push_back(ro);

        this->init(false);
    }
}
