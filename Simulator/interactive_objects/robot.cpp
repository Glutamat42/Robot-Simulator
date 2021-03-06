//
// Created by markus on 04.04.21.
//

#include "robot.h"
#include "../world/world.h"
#include "../../lib/helpers.h"
#include "CollisionData.h"

Robot::Robot(World *world) : CollidableCircle(world) {

}

Robot::Robot(std::string name,
             int radius,
             cv::Point2d start_pos,
             double start_orientation,
             World *world,
             double max_angle,
             double max_speed) : CollidableCircle(world) {
    this->name = name;
    this->radius = radius;
    this->pos = start_pos;
    this->orientation = start_orientation;
    this->max_angle = max_angle;
    this->max_speed = max_speed;

    // TODO: leaks memory, create a function returning a bool and deleting the temporary pointers
    if (this->collision_detection_map(&this->pos)) {
        throw std::invalid_argument("cant spawn in wall");
    }

    // TODO: leaks memory, create a function returning a bool and deleting the temporary pointers
    if (!this->collision_detection_objects(world->get_objects()).empty()) {
        throw std::invalid_argument("cant spawn in object");
    }
}

Robot::Robot(std::string name, int radius, World *world, double max_angle, double max_speed) : CollidableCircle(world) {
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

        // TODO: leaks memory, create a function returning a bool and deleting the temporary pointers -> see CollidableRay
        if (this->collision_detection_map(&start_pos)) {
            continue;
        }

        // TODO: leaks memory, create a function returning a bool and deleting the temporary pointers -> see CollidableRay
        if (!this->collision_detection_objects(world->get_objects()).empty()) {
            continue;
        }
        break;
    }

    this->pos = start_pos;
}

double Robot::get_orientation() {
    return this->orientation;
}

int Robot::add_sensor(SensorInterface *sensor) {
    this->sensors.push_back(sensor);
    return this->sensors.size();
}

std::vector<SensorInterface *> Robot::get_sensors() {
    return this->sensors;
}

void Robot::clearSensorsList(bool deletePointers) {
    if (deletePointers) {
        for (SensorInterface *sensor : this->sensors) {
            delete sensor;
        }
    }
    this->sensors.clear();
}


void Robot::set_turn_speed(double angle) {
    if (abs(angle) > this->max_angle) {
        this->move_angle = copysign(1.0, angle) * this->max_angle;
    } else {
        this->move_angle = angle;
    }
}

void Robot::set_speed(double speed) {
    if (abs(speed) > this->max_speed) {
        this->move_speed = sgn(speed) * this->max_speed;
    } else {
        this->move_speed = speed;
    }
}

void Robot::update() {
    if (!this->world) {
        throw std::logic_error("No world set, cant update!");
    }

    double desired_turn_angle = this->move_angle / GAME_TPS;
    double desired_move_distance = this->move_speed / GAME_TPS;

    // if target turn angle is set
    if (this->move_target_turn_angle != 0) {
        if (abs(desired_turn_angle) > abs(this->move_target_turn_angle)) {
            // finished turning for given angle (after this step)
            desired_turn_angle = this->move_target_turn_angle;
            this->move_angle = 0;
        }
        if (desired_turn_angle * this->move_target_turn_angle >= 0) { // both values have the same sign or desired_turn_angle is 0
            this->move_target_turn_angle -= desired_turn_angle;
        } else {
            this->move_target_turn_angle += desired_turn_angle;
        }
    }

    // if target travel/move distance is set
    if (this->move_target_distance != 0) {
        if (abs(desired_move_distance) > abs(this->move_target_distance)) {
            // finished moving for given distance (after this step)
            desired_move_distance = this->move_target_distance;
            this->move_speed = 0;
        }
        this->move_target_distance -= desired_move_distance;
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

        cv::Point2d currentCheckPoint = cv::Point2d(
                dx * step + this->pos.x,
                dy * step + this->pos.y
        );

        // check grayscaleMap collision
        WallPoint* collidedWallPoint = this->collision_detection_map(&currentCheckPoint);
        if (collidedWallPoint) {
            this->handleCollision(collidedWallPoint);
            delete collidedWallPoint;
            break;
        }
        delete collidedWallPoint;

        // check object collision
        std::vector<CollisionData *> collidedObjects = this->collision_detection_objects(world->get_objects(), &currentCheckPoint);
        if (!collidedObjects.empty()) {
            for (CollisionData *object : collidedObjects) {
                object->getObject()->handleCollision(this);
                this->handleCollision(object->getObject());
                delete object;
            }
            break;
        }

        last_collision_free_distance = step;
    } while (step < desired_move_distance);

    // update robot position
    this->pos = cv::Point2d(
            dx * last_collision_free_distance + this->pos.x,
            dy * last_collision_free_distance + this->pos.y
    );

    // save moved distance and angle
    // TODO: maybe implement measurement errors (standard deviation)
    this->last_tick_movement_angle = desired_turn_angle;
    this->last_tick_movement_distance = last_collision_free_distance;

    // update sensors
    // Updating them here causes some issues because that way the sensor update happens robot after robot. At the end of
    // the loop some robots will have moved since this sensor update. This can result in ugly situation where
    // (eg) a sensor ray goes through a robot because that robot updated its position after the sensor was updated.
//    for (SensorInterface *sensor : this->sensors) {
//        sensor->update_sensor_data();
//    }
}

void Robot::draw_robot(cv::Mat image) {
    circle(image, this->get_position(), customDrawRadius > 0 ? customDrawRadius : (int) this->radius, robotCircleColor, 1);
    if (!hideDirectionIndicator) {
        line(image,
             this->get_position(),
             this->get_position() + cv::Point2d(cos(this->get_orientation()) * this->radius,
                                                sin(this->get_orientation()) * this->radius),
             CV_RGB(0, 255, 0),
             1);
    }
}

std::string Robot::get_name() {
    return this->name;
}

void Robot::handleCollision(CollidableObject *object) {
//    std::cout << this->name << ": Collision detected" << std::endl;
}

double Robot::get_radius() {
    return this->radius;
}

double Robot::get_max_turn_rate() {
    return this->max_angle;
}

void Robot::setDrawOptions(cv::Scalar color, bool hideDirectionIndicator, int customRadius) {
    this->robotCircleColor = color;
    this->hideDirectionIndicator = hideDirectionIndicator;
    this->customDrawRadius = customRadius;
}

double Robot::get_max_move_speed() {
    return this->max_speed;
}
