//
// Created by markus on 05.04.21.
//

#ifndef MR_CPP_CODE_WORLD_H
#define MR_CPP_CODE_WORLD_H


#include "robot.h"

class World {
private:
    cv::Mat map;
    std::vector<Robot*> robots;

public:
    World(std::string map_filename);

    int add_robot(Robot* robot);

    bool check_collision(cv::Point2d point);

    cv::Point2d get_map_bounds();

    void show_map();
};


#endif //MR_CPP_CODE_WORLD_H
