//
// Created by markus on 12.04.21.
//

#ifndef MR_CPP_CODE_ROBOTCONTROLINTERFACE_H
#define MR_CPP_CODE_ROBOTCONTROLINTERFACE_H

class Sensor;

class RobotControlInterface {
public:
    virtual ~RobotControlInterface(){};

    virtual double get_radius() = 0;

    virtual void move(double pixel)= 0;

    virtual void turn(double angle)= 0;

    virtual std::vector<Sensor *> get_sensors()= 0;

    virtual void set_move_angle(double angle)= 0;

    virtual void set_speed(double speed)= 0;
};

#endif //MR_CPP_CODE_ROBOTCONTROLINTERFACE_H
