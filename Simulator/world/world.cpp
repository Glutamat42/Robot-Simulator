//
// Created by markus on 05.04.21.
//

#include "world.h"
#include "../constants.h"

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

    this->fastMap = FastMap(this->grayscaleMap);
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
    return this->fastMap.getPixel(point.x, point.y);
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

    for (MapObject* mapObject : this->mapObjects) {
        mapObject->draw(image);
    }

    cv::imshow(SIMULATOR_WINDOW_NAME + " - " + this->windowNameAppendix, image);
}

void World::add_object(CollidableObject *object) {
    Robot *casted_object = dynamic_cast<Robot *>(object);
    if (casted_object) {
        this->robots.push_back(casted_object);
    } else {
        this->arbitraryObjects.push_back(object);
    }
}

std::vector<CollidableObject *> World::get_objects() {
    std::vector<CollidableObject *> response;
    for(CollidableObject* o : this->robots) response.push_back(o);
    for(CollidableObject* o : this->arbitraryObjects) response.push_back(o);
    return response;
}


/* Returns a vector of only robots */
std::vector<Robot *> World::get_robots() {
    return this->robots;
}

/** clear objects list
 *
 * @param deletePointers if true also delete the objects instead of only clearing the list.
 */
void World::clearObjectsList(bool deletePointers) {
    if (deletePointers) {
        for (Robot *robot : this->robots) {
            robot->clearSensorsList(true);
        }
        for (auto o : this->robots) {
            delete o;
        }
        for (auto o : this->arbitraryObjects) {
            delete o;
        }
    }
    this->robots.clear();
    this->arbitraryObjects.clear();
}

void World::addMapObject(MapObject* object) {
    this->mapObjects.push_back(object);
}

/** will delete robots and sensors attached to the robots, but not pointers in those objects */
void World::clearRobotsList(bool deletePointers) {
    if (deletePointers) {
        for (Robot *robot : this->robots) {
            robot->clearSensorsList(true);
            delete robot;
        }
    }
    this->robots.clear();
}

/** will delete arbitraryObjects, but not pointers in those objects */
void World::clearArbitraryObjetsList(bool deletePointers) {
    if (deletePointers) {
        for (CollidableObject *object : this->arbitraryObjects) {
            delete object;
        }
    }
    this->arbitraryObjects.clear();
}

void World::addRobot(Robot *robot) {
    this->robots.push_back(robot);
}

void World::addArbitraryObject(CollidableObject *object) {
    this->arbitraryObjects.push_back(object);
}

std::vector<Robot *> World::getRobotsList() {
    return this->robots;
}

std::vector<CollidableObject *> World::getArbitraryObjectsList() {
    return this->arbitraryObjects;
}

void World::deleteRobotByIndex(long firstIndex, unsigned long count, bool deletePointers) {
    count = count - 1; // count is used as offset here
    if (firstIndex + count >= this->robots.size()) {
        throw std::invalid_argument("trying to delete elements out of bounds");
    }
    if (deletePointers) {
        for (long i = firstIndex; i < firstIndex + count; ++i) {
            this->robots[i]->clearSensorsList(true);
            delete this->robots[i];
        }
    }
    this->robots.erase(this->robots.begin() + firstIndex, this->robots.begin() + firstIndex + count);
}
