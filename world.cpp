//
// Created by markus on 05.04.21.
//

#include "world.h"
#include "sensor.h"
#include "constants.h"

/**
 *
 * @param map_filename filename
 */
World::World(std::string map_filename) {
    this->map = cv::imread(map_filename);
    if ((this->map.cols == 0) || (this->map.rows == 0)) {
        throw std::invalid_argument("Error! Could not read the image file: '" + map_filename + "'");
    }
}

int World::add_robot(Robot* robot) {
    this->robots.push_back(robot);
    return this->robots.size();
}

/** checks for collision of a point
 *
 * @param point
 * @return true: collision, false: no collision
 */
bool World::check_collision(cv::Point2d point) {
    cv::Vec3b pixel_color = this->map.at<cv::Vec3b>(point);
    return !((pixel_color.val[0] == 0) && (pixel_color.val[1] == 0) && (pixel_color.val[2] == 0));
}

cv::Point2d World::get_map_bounds() {
    return cv::Point2d(this->map.cols, this->map.rows);
}

void World::show_map() {
    cv::Mat image;
    this->map.copyTo(image);

    for (Robot* robot : this->robots) {
        robot->draw_robot(image);
        for (Sensor* sensor : robot->get_sensors()) {
            sensor->draw_sensor_data(image);
        }
    }

    cv::imshow(SIMULATOR_WINDOW_NAME, image);
}
