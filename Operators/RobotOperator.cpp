//
// Created by markus on 05.04.21.
//

#include "RobotOperator.h"
#include "../Simulator/constants.h"

RobotOperator::RobotOperator(RobotControlInterface *robot) {
    this->robot = robot;
}

std::vector<DistanceSensor *> RobotOperator::filter_for_distance_sensor(std::vector<SensorInterface *> sensors) {
    std::vector<DistanceSensor *> distance_sensors;
    for (SensorInterface *sensor: sensors) {
        DistanceSensor *casted_sensor = dynamic_cast<DistanceSensor *>(sensor);
        if (casted_sensor) distance_sensors.push_back(casted_sensor);
    }
    return distance_sensors;
}

