//
// Created by markus on 12.04.21.
//

#ifndef MR_CPP_CODE_SENSORINTERFACE_H
#define MR_CPP_CODE_SENSORINTERFACE_H


#include "../constants.h"

class World;
class Robot;

class SensorInterface {
protected:
    World* world;
    Robot* robot;
    double standardDeviation;

public:
    inline SensorInterface(World* world, Robot* robot, double standardDeviation = 1) {
        this->world = world;
        this->robot = robot;
        this->standardDeviation = standardDeviation * NOISE_MODIFIER;
    };

    virtual ~SensorInterface() = default;

    virtual void update_sensor_data(bool disable_object_collision_detection) = 0;

    virtual void draw_sensor_data(cv::Mat image) = 0;
};

#endif //MR_CPP_CODE_SENSORINTERFACE_H
