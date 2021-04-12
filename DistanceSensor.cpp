//
// Created by markus on 05.04.21.
//

#include "DistanceSensor.h"
#include "constants.h"

DistanceSensor::DistanceSensor(World *world, Robot *robot, double sensor_angle, double sensor_distance) : SensorInterface(world, robot) {
    this->world = world;
    this->robot = robot;
    this->sensor_angle = sensor_angle;
    this->sensor_max_distance = sensor_distance;
}

double DistanceSensor::get_sensor_angle() {
    return this->sensor_angle;
}

double DistanceSensor::get_sensor_max_distance() {
    return this->sensor_max_distance;
}

/** get the sensor value
 *
 * @return -1 if nothing detected (in sensor range), otherwise distance to object
 */
double DistanceSensor::get_sensor_value() {
    return this->sensor_data_value;
}

void DistanceSensor::update_sensor_data() {
    // abs sensor angle relative to world
    double sensor_dx = cos(this->robot->get_orientation() + this->sensor_angle);
    double sensor_dy = sin(this->robot->get_orientation() + this->sensor_angle);
    this->sensor_data_dxy = cv::Point2d(sensor_dx, sensor_dy);

    // start positions (vehicle border)
    double sensor_startx = this->robot->get_position().x + sensor_dx * this->robot->get_radius();
    double sensor_starty = this->robot->get_position().y + sensor_dy * this->robot->get_radius();
    this->sensor_data_abs_position = cv::Point2d(sensor_startx, sensor_starty);

    // default value if sensor does not detect anything (in range)
    this->sensor_data_value = -1;

    for (double i = 0; i < this->sensor_max_distance; i += CALCULATION_RESOLUTION) {
        double ray_x = sensor_startx + i * sensor_dx;
        double ray_y = sensor_starty + i * sensor_dy;

        if (this->world->check_collision(cv::Point2d(ray_x, ray_y))) {
            this->sensor_data_value = i;
            break;
        }
    }
}

void DistanceSensor::draw_sensor_data(cv::Mat image) {
    double draw_distance;
    if (this->sensor_data_value != -1) {
        draw_distance = this->sensor_data_value;
    } else { draw_distance = this->sensor_max_distance; }

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
    if (this->sensor_data_value == -1) return this->sensor_max_distance;
    return sensor_data_value;
}
