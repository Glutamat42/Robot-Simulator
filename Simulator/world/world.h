//
// Created by markus on 05.04.21.
//

#ifndef MR_CPP_CODE_WORLD_H
#define MR_CPP_CODE_WORLD_H


#include "../interactive_objects/robot.h"
#include "MapObject.h"

class World {
private:
    cv::Mat grayscaleMap;
    std::vector<CollidableObject *> objects;
    std::vector<MapObject*> mapObjects;
    std::string windowNameAppendix;
    cv::Point2i mapBounds;  // store in separate variable to increase performance when accessing grayscaleMap bounds

    std::vector<bool> fastMap;

    void setFastMapPixel(int x, int y, bool value) {
        fastMap[mapBounds.y * y + x] = value;
    }

    bool getFastMapPixel(int x, int y) {
        return fastMap[mapBounds.y * y + x];
    }

public:
    explicit World(std::string map_filename, std::string windowNameAppendix = "");

    std::vector<Robot *> get_robots();

    void clearObjectsList(bool deletePointers = false);

    bool check_collision(cv::Point2d point);

    cv::Point2d get_map_bounds();

    void show_map(bool hideSensors = false);

    void add_object(CollidableObject *object);

    std::vector<CollidableObject *> get_objects();

    void addMapObject(MapObject* object);
};


#endif //MR_CPP_CODE_WORLD_H