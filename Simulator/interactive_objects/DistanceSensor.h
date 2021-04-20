//
// Created by markus on 05.04.21.
//

#ifndef MR_CPP_CODE_DISTANCESENSOR_H
#define MR_CPP_CODE_DISTANCESENSOR_H


#include "../world.h"
#include "../interfaces/SensorInterface.h"
#include "interfaces/CollidableRay.h"

class DistanceSensor : public CollidableRay, public SensorInterface {
private:
    double sensor_angle;

    double sensor_data_value;
    cv::Point2d sensor_data_abs_position;
    cv::Point2d sensor_data_dxy;
public:
    DistanceSensor(World *world, Robot *robot, double sensor_angle, double sensor_distance);

    double get_sensor_angle();

    [[maybe_unused]] double get_sensor_max_distance();

    double get_sensor_value();

    double get_simplified_sensor_value();

    void update_sensor_data(bool disable_object_collision_detection) override;

    void draw_sensor_data(cv::Mat image) override;

    void handleCollision(CollidableObject *object) override;
};


#endif //MR_CPP_CODE_DISTANCESENSOR_H
