//
// Created by markus on 05.04.21.
//

#ifndef MR_CPP_CODE_WORLD_H
#define MR_CPP_CODE_WORLD_H


#include "interactive_objects/robot.h"

class World {
private:
    cv::Mat map;
    std::vector<CollidableObject*> objects;

public:
    World(std::string map_filename);

    std::vector<Robot*> get_robots();

    bool check_collision(cv::Point2d point);

    cv::Point2d get_map_bounds();

    void show_map();

    void add_object(CollidableObject* object);

    std::vector<CollidableObject*> get_objects();
};


#endif //MR_CPP_CODE_WORLD_H
