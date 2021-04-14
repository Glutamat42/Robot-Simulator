//
// Created by markus on 04.04.21.
//

#include "robot.h"
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
    }
}

Robot::Robot(std::string name, int radius, World *world, double max_angle, double max_speed) {
    this->name = name;
    this->radius = radius;
    this->world = world;
    this->max_angle = max_angle;
    this->max_speed = max_speed;
    this->orientation = fmod(rand(), (2 * M_PI)) - M_PI;

    cv::Point2d start_pos;
    while (true) {
        start_pos = cv::Point2d(
                // rand() % (width - radius) + radius / 2 with 3 decimals
                (rand() % (((int) world->get_map_bounds().x - radius) * 1000) / 1000) + radius / 2,
                (rand() % (((int) world->get_map_bounds().y - radius) * 1000) / 1000) + radius / 2
        );
        if (!this->collision_detection(start_pos)) {
            break;
        }
    }

    this->pos = start_pos;
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

int Robot::add_sensor(SensorInterface *sensor) {
    this->sensors.push_back(sensor);
    return this->sensors.size();
}

std::vector<SensorInterface *> Robot::get_sensors() {
    return this->sensors;
}

void Robot::set_turn_speed(double angle) {
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
    double desired_turn_angle = this->move_angle / GAME_TPS;
    double desired_move_distance = this->move_speed / GAME_TPS;

    // if target turn angle is set
    if (move_target_turn_angle != 0) {
        if (abs(desired_turn_angle) > abs(move_target_turn_angle)) {
            // finished turning for given angle (after this step)
            desired_turn_angle = move_target_turn_angle;
            this->move_angle = 0;
        }
        if (desired_turn_angle * move_target_turn_angle >=
            0) { // both values have the same sign or desired_turn_angle is 0
            move_target_turn_angle -= desired_turn_angle;
        } else {
            move_target_turn_angle += desired_turn_angle;
        }
    }

    // if target travel/move distance is set
    if (move_target_distance != 0) {
        if (abs(desired_move_distance) > abs(move_target_distance)) {
            // finished moving for given distance (after this step)
            desired_move_distance = move_target_distance;
            this->move_speed = 0;
        }
        move_target_distance -= desired_move_distance;
    }

    this->orientation = fmod((this->orientation + desired_turn_angle), (2 * M_PI));
    double dx = cos(this->orientation);
    double dy = sin(this->orientation);
    double last_collision_free_distance = 0.0;

    // collision detection for movement
    double step = 0.0;
    do {
        step += CALCULATION_RESOLUTION;
        if (step > desired_move_distance) step = desired_move_distance;

        if (this->collision_detection(
                cv::Point2d(
                        dx * step + this->pos.x,
                        dy * step + this->pos.y
                ))) {
            break;
        }
        last_collision_free_distance = step;
    } while (step < desired_move_distance);

    // update robot position
    this->pos = cv::Point2d(
            dx * last_collision_free_distance + this->pos.x,
            dy * last_collision_free_distance + this->pos.y
    );

    // update sensors
    for (SensorInterface *sensor : this->sensors) {
        sensor->update_sensor_data();
    }
}

void Robot::draw_robot(cv::Mat image) {
    circle(image, this->get_position(), (int) this->get_radius(), CV_RGB(255, 0, 0), 1);
    line(image,
         this->get_position(),
         this->get_position() + cv::Point2d(cos(this->get_orientation()) * this->get_radius(),
                                            sin(this->get_orientation()) * this->get_radius()),
         CV_RGB(0, 255, 0),
         1);
}
