//
// Created by markus on 12.04.21.
//

#ifndef MR_CPP_CODE_ROBOTCONTROLINTERFACE_H
#define MR_CPP_CODE_ROBOTCONTROLINTERFACE_H

#include "../../interfaces/SensorInterface.h"
#include <string>
#include <vector>

class RobotControlInterface {
public:
    virtual ~RobotControlInterface() = default;;

    virtual double get_radius() = 0;

    [[maybe_unused]] virtual void set_target_move_distance(double pixel) = 0;

    virtual double get_target_move_distance() = 0;

    virtual void set_target_turn_angle(double angle) = 0;

    virtual double get_target_turn_angle() = 0;

    virtual std::vector<SensorInterface *> get_sensors() = 0;

    virtual void set_turn_speed(double angle) = 0;

    virtual double get_turn_speed() = 0;

    virtual void set_speed(double speed) = 0;

    virtual double get_speed() = 0;

    virtual std::string get_name() = 0;

    virtual double get_last_tick_movement_distance() = 0;

    virtual double get_last_tick_movement_angle() = 0;

    virtual double get_max_turn_rate() = 0;

    virtual double get_max_move_speed() = 0;
};

#endif //MR_CPP_CODE_ROBOTCONTROLINTERFACE_H
