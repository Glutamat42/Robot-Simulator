//
// Created by markus on 04.04.21.
//

#ifndef MR_CPP_CODE_ROBOT_H
#define MR_CPP_CODE_ROBOT_H


#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <vector>
#include <string>

class World;

class Sensor;

class Robot {
private:
    std::string name;
    double radius;
    cv::Point2d pos;
    double orientation;
    World *world;
    double max_angle; // max angle per second
    double max_speed; // max speed in px/s
    double move_angle = 0;
    double move_speed = 0;
    std::vector<Sensor *> sensors;

    bool collision_detection(cv::Point2d pos);


public:
    Robot(std::string name, int radius, cv::Point2d start_pos, double start_orientation, World *world,
          double max_angle = M_PI / 6, double max_speed = 50.0);

    cv::Point2d get_position();

    double get_orientation();

    double get_radius();

    void move(double pixel);

    void turn(double angle);

    int add_sensor(Sensor *sensor);

    std::vector<Sensor *> get_sensors();

    void set_move_angle(double angle);

    void set_speed(double speed);

    void update();
};

#endif //MR_CPP_CODE_ROBOT_H
