//
// Created by markus on 12.04.21.
//

#ifndef MR_CPP_CODE_SENSORINTERFACE_H
#define MR_CPP_CODE_SENSORINTERFACE_H


class World;
class Robot;

class SensorInterface {
protected:
    World* world;
    Robot* robot;

public:
    inline SensorInterface(World* world, Robot* robot) {
        this->world = world;
        this->robot = robot;
    };

    virtual void update_sensor_data() = 0;

    virtual void draw_sensor_data(cv::Mat image) = 0;
};

#endif //MR_CPP_CODE_SENSORINTERFACE_H
