//
// Created by markus on 05.04.21.
//

#include "DistanceSensor.h"
#include "../helpers.h"
#include "CollisionData.h"

DistanceSensor::DistanceSensor(World *world, Robot *robot, double sensor_angle, double sensor_distance)
        : SensorInterface(world, robot),
          CollidableRay(robot->get_orientation() + sensor_angle, sensor_distance) {
    CollidableRay::world = SensorInterface::world;
    this->robot = robot;
    this->sensor_angle = sensor_angle;
}

double DistanceSensor::get_sensor_angle() {
    return this->sensor_angle;
}

double DistanceSensor::get_sensor_max_distance() {
    return this->length;
}

/** get the sensor value
 *
 * @return -1 if nothing detected (in sensor range), otherwise distance to object
 */
double DistanceSensor::get_sensor_value() {
    return this->sensor_data_value;
}

void DistanceSensor::update_sensor_data() {
    // update sensor angle (in case robot turned)
    this->angle = this->robot->get_orientation() + this->sensor_angle;


    // calculate exact sensor location
    // abs. sensor angle relative to world
    double sensor_dx = cos(this->angle);
    double sensor_dy = sin(this->angle);
    this->sensor_data_dxy = cv::Point2d(sensor_dx, sensor_dy);

    // start positions (vehicle border)
    double sensor_startx = this->robot->get_position().x + sensor_dx * this->robot->get_radius();
    double sensor_starty = this->robot->get_position().y + sensor_dy * this->robot->get_radius();
    this->sensor_data_abs_position = cv::Point2d(sensor_startx, sensor_starty);
    // save sensor location
    this->pos = cv::Point2d(sensor_startx, sensor_starty);


    // default value if sensor does not detect anything (in range)
    this->sensor_data_value = -1;

    double distanceToCollision;
    auto* pointOfCollision = dynamic_cast<WallPoint *>(this->collision_detection_map(nullptr, &distanceToCollision));
    if (pointOfCollision) {
        this->sensor_data_value = distanceToCollision;

        // add noise
        if (this->inaccuracy != 0) {
            double noise_multiplier = get_random_percentage(this->inaccuracy);
            this->sensor_data_value *= (1 + noise_multiplier);
        }
    }

    // Object collision
    std::vector<CollidableObject*> objectsWithoutMe;
    for (CollidableObject* object : SensorInterface::world->get_objects()) {
        if (object != this->robot) {
            objectsWithoutMe.push_back(object);
        }
    }
    std::vector<CollisionData *> collidedObjects = collision_detection_objects(objectsWithoutMe);
    for (CollisionData* collisionData : collidedObjects) {
        cv::Point2d distanceVector = collisionData->getPoint() - this->pos;
        double distance = sqrt(distanceVector.x * distanceVector.x + distanceVector.y * distanceVector.y);
        if (distance < this->sensor_data_value || this->sensor_data_value == -1) {
            this->sensor_data_value = distance;
        }
    }
}

void DistanceSensor::draw_sensor_data(cv::Mat image) {
    double draw_distance;
    if (this->sensor_data_value != -1) {
        draw_distance = this->sensor_data_value;
    } else { draw_distance = this->length; }

    line(image,
         this->sensor_data_abs_position,
         cv::Point2d(this->sensor_data_abs_position.x + this->sensor_data_dxy.x * draw_distance,
                     this->sensor_data_abs_position.y + this->sensor_data_dxy.y * draw_distance),
         CV_RGB(0, 0, 255),
         1);
}

/** provides simplified access to sensor value by replacing -1 (nothing detected) with max sensor distance
 *
 * @return measured distance
 */
double DistanceSensor::get_simplified_sensor_value() {
    if (this->sensor_data_value == -1) return this->length;
    return sensor_data_value;
}

/** Maybe add an update cycle if something hits the sensor ray, but currently the sensor ray isn't known to the map
 * and because of that wont be observed during collision checks
 */
void DistanceSensor::handleCollision(CollidableObject *object) {}
