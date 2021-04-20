//
// Created by markus on 20.04.21.
//

#include "BasicWithNovelty.h"
#include "../Simulator/helpers.h"


BasicWithNovelty::BasicWithNovelty(RobotControlInterface *robot) : RobotOperator(robot) {
    this->robot->set_speed(150);

    this->distance_sensor_history.reserve(distance_sensor_history_size);
}

void BasicWithNovelty::update() {
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
        this->robot->set_target_turn_angle(std::fmod(((double) rand() / 1000), 2 * M_PI) - M_PI);
        this->robot->set_turn_speed(this->robot->get_max_turn_rate() * sgn(this->robot->get_target_turn_angle()));
        this->robot->set_speed(0);
        return;
    }

    std::vector<DistanceSensor *> distance_sensors = this->filter_for_distance_sensor(this->robot->get_sensors());

//    DistanceSensor *best_sensor = distance_sensors[0];
    DistanceSensor *worst_sensor = distance_sensors[0];
    DistanceSensor *worst_left_sensor = nullptr;
    DistanceSensor *worst_right_sensor = nullptr;
    for (DistanceSensor *sensor : distance_sensors) {
        if (sensor->get_sensor_angle() < 0) {
            if (!worst_left_sensor || (sensor->get_simplified_sensor_value() < worst_left_sensor->get_simplified_sensor_value())) {
                worst_left_sensor = sensor;
            }
        } else {
            if (!worst_right_sensor || (sensor->get_simplified_sensor_value() < worst_right_sensor->get_simplified_sensor_value())) {
                worst_right_sensor = sensor;
            }
        }

//        if (sensor->get_simplified_sensor_value() > best_sensor->get_simplified_sensor_value()) best_sensor = sensor;
        if (sensor->get_simplified_sensor_value() < worst_sensor->get_simplified_sensor_value()) worst_sensor = sensor;
    }


    // if one the worst left and right sensors output roughtly the same value i expect to be in the middle between two obstacles
    if (abs(worst_left_sensor->get_simplified_sensor_value() - worst_right_sensor->get_simplified_sensor_value()) < 20 && worst_sensor->get_simplified_sensor_value() > 15) {
        this->robot->set_turn_speed(0);
        return;
    }

    double newTurnSpeed = worst_sensor->get_sensor_angle() * -1 * 50 * (1 / worst_sensor->get_simplified_sensor_value());
    if (abs(newTurnSpeed - this->robot->get_turn_speed()) > 0.2) {
        this->robot->set_turn_speed(newTurnSpeed);
    }

}

bool BasicWithNovelty::novelty_detection() {
    std::vector<DistanceSensor *> distance_sensors = this->filter_for_distance_sensor(this->robot->get_sensors());

    // create vector containing sensor values
    std::vector<int> cur_sensor_values;
    bool allInfinite = true;
    for (DistanceSensor *sensor : distance_sensors) {
        cur_sensor_values.push_back((int) sensor->get_sensor_value());
        if (sensor->get_sensor_value() != -1) {
            allInfinite = false;
        }
    }

    if (allInfinite) {
        // did not detect anything useful, if every sensor returns -1 the robots is simple anywhere where he can't detect any walls
        return false;
    }


    // first check if robot was already here
    bool was_here_before = false;
    for (std::vector<int> loop_sensor_values : distance_sensor_history) {
        if (loop_sensor_values == cur_sensor_values) {
            was_here_before = true;
        }
    }

    // modify counters depending on whether the current sensor pattern was seen before
    if (was_here_before) {
        novelty_weight++;
        novelty_score += novelty_weight;
    } else {
        novelty_weight = 0;
        novelty_score /= 2;
    }

    // and then add the current point to the list
    if (distance_sensor_history.size() >= distance_sensor_history_size) {
        distance_sensor_history.at(distance_sensor_history_index) = cur_sensor_values;
    } else {
        distance_sensor_history.push_back(cur_sensor_values);
    }
    distance_sensor_history_index = (distance_sensor_history_index + 1) % distance_sensor_history_size;


    // handle threshold exceeded
    if (novelty_score > NOVELTY_THRESHOLD) {
        novelty_weight = 0;
        novelty_score = 0;
        return true;
    }
    return false;
}
