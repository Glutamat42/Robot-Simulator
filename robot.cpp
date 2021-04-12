//
// Created by markus on 04.04.21.
//

#include "robot.h"
#include "sensor.h"
#include "world.h"
#include "constants.h"

Robot::Robot(std::string name,
             int radius,
             cv::Point2d start_pos,
             double start_orientation,
             World *world,
             double max_angle,
             double max_speed) {
    this->name = name;
    this->radius = radius;
    this->pos = start_pos;
    this->orientation = start_orientation;
    this->world = world;
    this->max_angle = max_angle;
    this->max_speed = max_speed;

    if (this->collision_detection(this->pos)) {
        throw std::invalid_argument("cant spawn in wall");
    } else {
        std::cout << "i spawned :)";
    }
}

cv::Point2d Robot::get_position() {
    return this->pos;
}

double Robot::get_orientation() {
    return this->orientation;
}

double Robot::get_radius() {
    return this->radius;
}

void Robot::move(double pixel) {

}

void Robot::turn(double angle) {

}

/** check for collision robot <-> wall
 *
 * @param pos center
 * @return true if collision, false if not
 */
bool Robot::collision_detection(cv::Point2d pos) {
    // check map bounds
    if (pos.x + radius > this->world->get_map_bounds().x
        || pos.y + radius > this->world->get_map_bounds().y
        || pos.x - radius < 0
        || pos.y - radius < 0) {
        return true;
    }

    // check collisions on map
    for (int x = floor(pos.x) - this->radius; x <= ceil(pos.x) + radius; ++x) {
        for (int y = floor(pos.y) - this->radius; y <= ceil(pos.y) + radius; ++y) {
            if ((x - pos.x) * (x - pos.x) + (y - pos.y) * (y - pos.y) <= this->radius * this->radius) {
                if (this->world->check_collision(cv::Point2d(x, y))) return true;
            }
        }
    }

    // no collision
    return false;
}

int Robot::add_sensor(Sensor *sensor) {
    this->sensors.push_back(sensor);
    return this->sensors.size();
}

std::vector<Sensor *> Robot::get_sensors() {
    return this->sensors;
}

void Robot::set_move_angle(double angle) {
    if (abs(angle) > this->max_angle) {
        this->move_angle = copysign(1.0, angle) * this->max_angle;
    } else {
        this->move_angle = angle;
    }
}

void Robot::set_speed(double speed) {
    if (speed > this->max_speed) {
        this->move_speed = this->max_speed;
    } else {
        this->move_speed = speed;
    }
}

void Robot::update() {
    this->orientation = fmod((this->orientation + this->move_angle / TARGET_TPS), (2 * M_PI));
    double travel_distance = this->move_speed / TARGET_TPS;
    double dx = cos(this->orientation);
    double dy = sin(this->orientation);
    double last_collision_free_distance = 0.0;

    // collision detection for movement
    double step = 0.0;
    do {
        step += CALCULATION_RESOLUTION;
        if (step > travel_distance) step = travel_distance;

        if (this->collision_detection(
                cv::Point2d(
                        dx * step + this->pos.x,
                        dy * step + this->pos.y
                ))) {
            break;
        }
        last_collision_free_distance = step;
    } while (step < travel_distance);

    // move
    this->pos = cv::Point2d(
            dx * last_collision_free_distance + this->pos.x,
            dy * last_collision_free_distance + this->pos.y
    );

    // update sensors
    for (Sensor *sensor : this->sensors) {
        sensor->update_sensor_data();
    }
}


