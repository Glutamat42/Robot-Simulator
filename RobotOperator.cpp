//
// Created by markus on 05.04.21.
//

#include "RobotOperator.h"
#include "constants.h"

RobotOperator::RobotOperator(RobotControlInterface *robot) {
    this->robot = robot;

    this->robot->set_speed(150);

    this->distance_sensor_history.reserve(distance_sensor_history_size);
}

void RobotOperator::update() {
    if (unstucking) {
        if (this->robot->get_target_turn_angle() == 0) {
            this->unstucking = false;
            std::cout << this->robot->get_name() << ": finished unstucking" << std::endl;
            this->robot->set_speed(150);
        } else {
            return;
        }
    }

    if (this->novelty_detection()) {
        std::cout << this->robot->get_name() << ": Looks like i'm stuck :(" << std::endl;
        this->unstucking = true;
        this->robot->set_target_turn_angle(std::fmod(((double) rand() / 1000), 2 * M_PI));
        this->robot->set_speed(0);
    }

    std::vector<DistanceSensor *> distance_sensors = this->filter_for_distance_sensor(this->robot->get_sensors());

    DistanceSensor *best_sensor = distance_sensors[0];
    DistanceSensor *worst_sensor = distance_sensors[0];
    for (DistanceSensor *sensor : distance_sensors) {
        if (sensor->get_simplified_sensor_value() > best_sensor->get_simplified_sensor_value()) best_sensor = sensor;
        if (sensor->get_simplified_sensor_value() < worst_sensor->get_simplified_sensor_value()) worst_sensor = sensor;
    }

    this->robot->set_turn_speed(
            worst_sensor->get_sensor_angle() * -1 * 50 * (1 / worst_sensor->get_simplified_sensor_value()));
}

std::vector<DistanceSensor *> RobotOperator::filter_for_distance_sensor(std::vector<SensorInterface *> sensors) {
    std::vector<DistanceSensor *> distance_sensors;
    for (SensorInterface *sensor: sensors) {
        DistanceSensor *casted_sensor = dynamic_cast<DistanceSensor *>(sensor);
        if (casted_sensor) distance_sensors.push_back(casted_sensor);
    }
    return distance_sensors;
}

bool RobotOperator::novelty_detection() {
    std::vector<DistanceSensor *> distance_sensors = this->filter_for_distance_sensor(this->robot->get_sensors());

    // create vector containing sensor values
    std::vector<int> cur_sensor_values;
    for (DistanceSensor *sensor : distance_sensors) {
        cur_sensor_values.push_back((int) sensor->get_sensor_value());
    }

    // first check if robot was already here
    bool was_here_before = false;
    for (std::vector<int> loop_sensor_values : distance_sensor_history) {
        if (loop_sensor_values == cur_sensor_values) {
            was_here_before = true;
        }
    }

    // and then add the current point to the list
    if (distance_sensor_history.size() >= distance_sensor_history_size) {
        distance_sensor_history.at(distance_sensor_history_index) = cur_sensor_values;
    } else {
        distance_sensor_history.push_back(cur_sensor_values);
    }
    distance_sensor_history_index = (distance_sensor_history_index + 1) % distance_sensor_history_size;

    // modify counters depending on whether the current sensor pattern was seen before
    if (was_here_before) {
        novelty_weight++;
        novelty_score += novelty_weight;
    } else {
        novelty_weight = 0;
        novelty_score /= 2;
    }

    // handle threshold exceeded
    if (novelty_score > NOVELTY_THRESHOLD) {
        novelty_weight = 0;
        novelty_score = 0;
        return true;
    }
    return false;
}
