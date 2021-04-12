//
// Created by markus on 05.04.21.
//

#include "RobotOperator.h"

RobotOperator::RobotOperator(RobotControlInterface *robot) {
    this->robot = robot;

    this->robot->set_speed(150);
}

void RobotOperator::update() {
    Sensor* best_sensor = robot->get_sensors()[0];
    Sensor* worst_sensor = robot->get_sensors()[0];
    for (Sensor* sensor : this->robot->get_sensors()) {
        if (sensor->get_simplified_sensor_value() > best_sensor->get_simplified_sensor_value()) best_sensor = sensor;
        if (sensor->get_simplified_sensor_value() < worst_sensor->get_simplified_sensor_value()) worst_sensor = sensor;
    }

    this->robot->set_move_angle(worst_sensor->get_sensor_angle() * -1 * 50 * (1/worst_sensor->get_simplified_sensor_value()));
}
