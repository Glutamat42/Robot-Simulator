//
// Created by markus on 12.04.21.
//

#ifndef MR_CPP_CODE_SENSORINTERFACE_H
#define MR_CPP_CODE_SENSORINTERFACE_H

#include "../constants.h"
#include <opencv2/core/types.hpp>

class World;
class Robot;

enum class SensorType {
    distance
};

class SensorInterface {
protected:
    World* world;
    Robot* robot;
    double standardDeviation;
    SensorType sensorType;

public:
    inline SensorInterface(World* world, Robot* robot, SensorType sensorType, double standardDeviation = 1) {
        this->world = world;
        this->robot = robot;
        this->sensorType = sensorType;
        this->standardDeviation = standardDeviation * NOISE_MODIFIER;
    };

    virtual ~SensorInterface() = default;

    virtual void update_sensor_data(bool disable_object_collision_detection) = 0;

    virtual void draw_sensor_data(cv::Mat image) = 0;

    SensorType getSensorType() { return this->sensorType; }
};

#endif //MR_CPP_CODE_SENSORINTERFACE_H
