//
// Created by markus on 12.04.21.
//

#ifndef MR_CPP_CODE_ROBOTCONTROLINTERFACE_H
#define MR_CPP_CODE_ROBOTCONTROLINTERFACE_H

#include "../../interfaces/SensorInterface.h"

class RobotControlInterface {
public:
    virtual ~RobotControlInterface() = default;;

    virtual double get_radius() = 0;

    virtual void set_target_move_distance(double pixel) = 0;

    virtual double get_target_move_distance() = 0;

    virtual void set_target_turn_angle(double angle) = 0;

    virtual double get_target_turn_angle() = 0;

    virtual std::vector<SensorInterface *> get_sensors() = 0;

    virtual void set_turn_speed(double angle) = 0;

    virtual double get_turn_speed() = 0;

    virtual void set_speed(double speed) = 0;

    virtual double get_speed() = 0;

    virtual std::string get_name() = 0;
};

#endif //MR_CPP_CODE_ROBOTCONTROLINTERFACE_H
