//
// Created by markus on 21.06.21.
//

#include <random>
#include "QLearningOperator.h"

QLearningOperator::QLearningOperator(RobotControlInterface *robot, std::map<std::vector<int>, std::map<std::array<int, 2>, double>> qMap, bool train) : RobotOperator(robot) {
    this->train = train;
    this->q_map = qMap;
    this->distanceSensors = DistanceSensor::filter_for_distance_sensor(this->robot->get_sensors());
    this->initialPosition = ((Robot*) this->robot)->get_position(); // need cheat mode here to calculate travelled distance since spawn -> required only for training / metrics
}

void QLearningOperator::update() {
    // this happens on every update which is quite sub optimal
    std::random_device rd;
    std::default_random_engine mt(rd());
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    std::uniform_int_distribution<int> dist_speed(0, this->mapping_speed.size()-1);
    std::uniform_int_distribution<int> dist_angle(0, this->mapping_angle.size()-1);

    // start of the actual update method
    int speed;
    int angle;
    this->stateBeforeUpdate = this->getSensorValuesAsBin();

    if (dist(mt) < this->EXPLORATION_VS_EXPLOITATION) {
        // random action -> explore
        speed = dist_speed(mt);
        angle = dist_angle(mt);
    } else {
        // do best action according to policies
        std::array<int, 2> bestAction = std::get<0>(this->getBestAction(this->stateBeforeUpdate));
        speed = bestAction[0];
        angle = bestAction[1];
    }

    // do action
    this->robot->set_speed(this->mapping_speed[speed]);
    this->robot->set_turn_speed(this->mapping_angle[angle]);

    // save action for post update function
    this->chosenAction = {speed, angle};
}

void QLearningOperator::afterUpdate() {
    // reward for moving away from the spawn point
//    Robot* r = (Robot*) this->robot; // need cheat mode here to calculate travelled distance since spawn -> required only for training / metrics
//    cv::Point2d diff = this->initialPosition - r->get_position();
//    double distanceFromSpawn = cv::sqrt(diff.x*diff.x + diff.y*diff.y);
//    double reward = distanceFromSpawn;
//    this->collectedReward = distanceFromSpawn;

    // reward for moved distance and punishment for turning
    double reward = this->robot->get_last_tick_movement_distance() - std::abs(this->robot->get_last_tick_movement_angle());
    if (robot->get_last_tick_movement_distance() < 10 / GAME_TPS) {
        reward = -100;
    }
    this->collectedReward += reward;

    if (this->train) {
        // here happens Q-Learning
        double oldReward = this->q_map[this->stateBeforeUpdate][this->chosenAction];
        std::vector<int> newSensorBins = this->getSensorValuesAsBin();
        double futureReward = std::get<1>(this->getBestAction(newSensorBins));

        double updatedReward = oldReward + this->LR * (reward + this->DISCOUNTING_FACTOR * futureReward - oldReward);
        this->q_map[this->stateBeforeUpdate][this->chosenAction] = updatedReward;
    }
}

std::vector<int> QLearningOperator::getSensorValuesAsBin() {
    std::vector<int> sensData;

    for (DistanceSensor *s : this->distanceSensors) {
        double sensValue = s->get_sensor_value();
        for (int i = 0; i < this->bins_sensor.size(); ++i) {
            if (sensValue < this->bins_sensor[i]) {
                sensData.push_back(i);
                break;
            }
        }
    }

    return sensData;
}

/**
 *
 * @param sensorBins
 * @return best action <speed, angle>, reward for this action
 */
std::tuple<std::array<int, 2>,double> QLearningOperator::getBestAction(std::vector<int> sensorBins) {
    // make sure state is in q_map
    if (this->q_map.count(sensorBins) == 0) {
        std::map<std::array<int, 2>, double> actionMap;
        for (int speed_index = 0; speed_index < this->mapping_speed.size(); ++speed_index) {
            for (int angle_index = 0; angle_index < this->mapping_angle.size(); ++angle_index) {
                actionMap.insert(std::make_pair(std::array<int, 2>({speed_index, angle_index}), 0)); // initializing with 0, maybe use random values
            }
        }

        this->q_map.insert(std::make_pair(sensorBins, actionMap));
    }

    // get the currently relevant dataset for which we want to find the best entry
    std::map<std::array<int, 2>, double> currentState = this->q_map[sensorBins];
    // initialize variables
    std::array<int, 2> bestAction = currentState.begin()->first;
    double bestActionRevenue = currentState.begin()->second;

    // find best entry
    for (auto & it : currentState) {
        if (it.second > bestActionRevenue) {
            bestActionRevenue = it.second;
            bestAction = it.first;
        }
    }

    return std::make_tuple(bestAction, bestActionRevenue);
}

std::map<std::vector<int>, std::map<std::array<int, 2>, double>> QLearningOperator::getQMap() {
    return this->q_map;
}

double QLearningOperator::getCollectedReward() {
    return this->collectedReward;
}
