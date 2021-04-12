//
// Created by markus on 04.04.21.
//

#ifndef MR_CPP_CODE_ROBOT_H
#define MR_CPP_CODE_ROBOT_H


#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <vector>
#include <string>
#include "interfaces/RobotControlInterface.h"

class World;

class Sensor;

class Robot : public RobotControlInterface {
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

    /** specify distance until robot should stop moving. It'll move with the specified speed (set_speed)
     * If this function is not called the robot will move until stopped by manually setting move_speed to 0
     * or setting a distance limit by this function
     *
     * TODO: implement
     *
     * @param pixel
     */
    void move(double pixel);

    /** same as move(...) but for angle
     *
     * TODO: implement
     *
     * @param angle
     */
    void turn(double angle);

    int add_sensor(Sensor *sensor);

    std::vector<Sensor *> get_sensors();

    void set_move_angle(double angle);

    void set_speed(double speed);

    void update();

    void draw_robot(cv::Mat image);
};

#endif //MR_CPP_CODE_ROBOT_H
