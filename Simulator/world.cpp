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
    this->grayscaleMap = cv::imread(map_filename, cv::IMREAD_GRAYSCALE);
    if ((this->grayscaleMap.cols == 0) || (this->grayscaleMap.rows == 0)) {
        throw std::invalid_argument("Error! Could not read the image file: '" + map_filename + "'");
    }
    this->windowNameAppendix = windowNameAppendix;
    this->mapBounds = cv::Point2d(this->grayscaleMap.cols, this->grayscaleMap.rows);

    this->fastMap = std::vector<bool>(this->grayscaleMap.rows * this->grayscaleMap.cols);
    for (int y = 0; y < this->mapBounds.y; y++) {
        for (int x = 0; x < this->mapBounds.x; x++) {
            this->setFastMapPixel(x, y, this->grayscaleMap.at<uchar>(y, x) != 0);
        }
    }
}

/** checks for collision of a point
 *
 * @param point
 * @return true: collision, false: no collision
 */
bool World::check_collision(cv::Point2d point) {
    // check grayscaleMap bounds
    if (point.x > this->mapBounds.x
        || point.y > this->mapBounds.y
        || point.x < 0
        || point.y < 0) {
        return true;
    }

    // check walls
    return this->getFastMapPixel(point.x, point.y);
}

cv::Point2d World::get_map_bounds() {
    return this->mapBounds;
}

void World::show_map(bool hideSensors) {
    cv::Mat image;
    cv::cvtColor(this->grayscaleMap, image, cv::COLOR_GRAY2RGB);

    for (Robot *robot : this->get_robots()) {
        robot->draw_robot(image);
        if (!hideSensors) {
            for (SensorInterface *sensor : robot->get_sensors()) {
                sensor->draw_sensor_data(image);
            }
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

/** clear objects list
 *
 * @param deletePointers if true also delete the objects instead of only clearing the list.
 */
void World::clearObjectsList(bool deletePointers) {
    if (deletePointers) {
        for (Robot *robot : this->get_robots()) {
            robot->clearSensorsList(true);
        }
        for (auto o : this->objects) {
            delete o;
        }
    }
    this->objects.clear();
}
