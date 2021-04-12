//
// Created by markus on 05.04.21.
//

#ifndef MR_CPP_CODE_SENSOR_H
#define MR_CPP_CODE_SENSOR_H


#include "world.h"

class Sensor {
private:
    World* world;
    Robot* robot;
    double sensor_angle;
    double sensor_max_distance;

    double sensor_data_value;
    cv::Point2d sensor_data_abs_position;
    cv::Point2d sensor_data_dxy;
public:
    Sensor(World* world, Robot* robot, double sensor_angle, double sensor_distance);

    double get_sensor_angle();

    double get_sensor_max_distance();

    double get_sensor_value();

    double get_simplified_sensor_value();

    void update_sensor_data();

    void draw_sensor_data(cv::Mat image);
};


#endif //MR_CPP_CODE_SENSOR_H
