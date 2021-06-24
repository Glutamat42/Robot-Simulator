//
// Created by markus on 21.06.21.
//

#include "QLearningScenario.h"

void QLearningScenario::setUp() {
    // setup logfile (log training progress)
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::stringstream dateStream;
    dateStream << std::put_time(&tm, "%d-%m-%Y_%H-%M-%S");
    std::string dateTimeString = dateStream.str();
    this->logfile.open("logs/" + dateTimeString + "_q-learning.csv");

    // setup actual scenario (for the first epoch)
    this->resetScenario();
}

void QLearningScenario::afterUpdate() {
    this->cur_step++;
    if (this->cur_step == this->MAX_STEPS_PER_EPISODE) {
        this->logfile << this->cur_epoch << "," << this->ro->getCollectedReward() << std::endl;

        if (this->cur_epoch == this->NR_EPISODES) {
            // training finished
        }

        this->cur_step = 0;
        this->cur_epoch++;
        this->qMap = this->ro->getQMap();
        this->resetScenario();
    }
}

void QLearningScenario::resetScenario() {
    std::string mapName = "assets/maps/world1.png";
    this->world = new World(mapName);


//    Robot *r = new Robot(std::string("r1"), 8, cv::Point2d(310.0, 300.0), 1.0 * M_PI, world, M_PI / 2, 40);
    Robot *r = new Robot(std::string("r1"), 8, world, M_PI / 2, 40);
    DistanceSensor *s1 = new DistanceSensor(world, r, 0.1 * M_PI, 200);
    DistanceSensor *s2 = new DistanceSensor(world, r, -0.1 * M_PI, 200);
    DistanceSensor *s3 = new DistanceSensor(world, r, 0.25 * M_PI, 200);
    DistanceSensor *s4 = new DistanceSensor(world, r, -0.25 * M_PI, 200);
    r->add_sensor(s1);
    r->add_sensor(s2);
    r->add_sensor(s3);
    r->add_sensor(s4);

    world->add_object(r);
    this->ro = new QLearningOperator(r, this->qMap, true);

    sensors.push_back(s1);
    sensors.push_back(s2);
    sensors.push_back(s3);
    sensors.push_back(s4);
    robots.push_back(r);
    robotOperators.push_back(this->ro);

    this->init(false);
}

QLearningScenario::~QLearningScenario() {
    this->logfile.close();
}
