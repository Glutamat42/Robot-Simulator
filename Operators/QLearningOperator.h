//
// Created by markus on 21.06.21.
//

#ifndef MR_CPP_CODE_QLEARNINGOPERATOR_H
#define MR_CPP_CODE_QLEARNINGOPERATOR_H


#include "RobotOperator.h"

class QLearningOperator : public RobotOperator {
private:
    std::array<double,7> bins_sensor = {0, 10, 25 ,50, 75, 100, INT32_MAX};  // upper bounds, (x < bins_sensor)
    std::array<double,5> mapping_speed = {-10, 0, 10, 25, 50};  // upper bounds, (x < bins_sensor)
    std::array<double,5> mapping_angle = {- M_PI / 2, -M_PI / 4, 0, M_PI / 4, M_PI / 2};  // upper bounds, (x < bins_sensor)

    double EXPLORATION_VS_EXPLOITATION = 0.05;
    double LR = 0.2;
    double DISCOUNTING_FACTOR = 0.9; // gamma

    bool train;

    std::vector<DistanceSensor *> distanceSensors;
    std::vector<double> q_score;
    std::map<std::vector<int>, std::map<std::array<int, 2>, double>> q_map; // value, actions; actions = speed, angle, reward
    std::vector<int> stateBeforeUpdate;
    std::array<int,2> chosenAction;
    double collectedReward = 0;
    cv::Point2d initialPosition;

    std::tuple<std::array<int, 2>,double> getBestAction(std::vector<int> sensorBins);
    std::vector<int> getSensorValuesAsBin();
public:
    QLearningOperator(RobotControlInterface *robot, std::map<std::vector<int>, std::map<std::array<int, 2>, double>> qMap, bool train=false);

    void update() override;

    void afterUpdate() override;

    std::map<std::vector<int>, std::map<std::array<int, 2>, double>> getQMap();

    double getCollectedReward();
};


#endif //MR_CPP_CODE_QLEARNINGOPERATOR_H
