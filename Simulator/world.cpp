//
// Created by markus on 05.04.21.
//

#include "world.h"
#include "constants.h"

/**
 *
 * @param map_filename filename
 */
World::World(std::string map_filename, std::string windowNameAppendix) {
    this->map = cv::imread(map_filename);
    if ((this->map.cols == 0) || (this->map.rows == 0)) {
        throw std::invalid_argument("Error! Could not read the image file: '" + map_filename + "'");
    }
    this->windowNameAppendix = windowNameAppendix;
}

/** checks for collision of a point
 *
 * @param point
 * @return true: collision, false: no collision
 */
bool World::check_collision(cv::Point2d point) {
    // check map bounds
    if (point.x > this->get_map_bounds().x
        || point.y > this->get_map_bounds().y
        || point.x < 0
        || point.y < 0) {
        return true;
    }

    // check walls
    cv::Vec3b pixel_color = this->map.at<cv::Vec3b>(point);
    return !((pixel_color.val[0] == 0) && (pixel_color.val[1] == 0) && (pixel_color.val[2] == 0));
}

cv::Point2d World::get_map_bounds() {
    return cv::Point2d(this->map.cols, this->map.rows);
}

void World::show_map() {
    cv::Mat image;
    this->map.copyTo(image);

    for (Robot* robot : this->get_robots()) {
        robot->draw_robot(image);
        for (SensorInterface* sensor : robot->get_sensors()) {
            sensor->draw_sensor_data(image);
        }
    }

    cv::imshow(SIMULATOR_WINDOW_NAME + " - " + this->windowNameAppendix, image);
}

void World::add_object(CollidableObject *object) {
    this->objects.push_back(object);
}

std::vector<CollidableObject *> World::get_objects() {
    return this->objects;
}

std::vector<Robot *> World::get_robots() {
    std::vector<Robot *> robots;
    for (CollidableObject *object: this->objects) {
        Robot *casted_object = dynamic_cast<Robot *>(object);
        if (casted_object) robots.push_back(casted_object);
    }
    return robots;

//    std::vector<DistanceSensor *> distance_sensors;
//    for (SensorInterface *sensor: sensors) {
//        DistanceSensor *casted_sensor = dynamic_cast<DistanceSensor *>(sensor);
//        if (casted_sensor) distance_sensors.push_back(casted_sensor);
//    }
//    return distance_sensors;
}

void World::clearObjectsList() {
    this->objects.clear();
}
