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
#include "interfaces/SensorInterface.h"

class World;

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
    double move_target_distance = 0; // will set_target_move_distance for this amount of pixels and then stop
    double move_target_turn_angle = 0; // will set_target_turn_angle for this angle and then stop turning
    std::vector<SensorInterface *> sensors;

    bool collision_detection(cv::Point2d pos);


public:
    Robot(std::string name, int radius, cv::Point2d start_pos, double start_orientation, World *world,
          double max_angle = M_PI / 6, double max_speed = 50.0);

    cv::Point2d get_position();

    double get_orientation();

    double get_radius();

    /** specify distance until robot should stop moving. It'll set_target_move_distance with the specified speed (set_speed)
     * If this function is not called the robot will set_target_move_distance until stopped by manually setting move_speed to 0
     * or setting a distance limit by this function
     *
     * TODO: implement
     *
     * @param pixel
     */
    void set_target_move_distance(double pixel) {
        this->move_target_distance = pixel;
    };

    double get_target_move_distance() {
        return move_target_distance;
    }

    /** same as set_target_move_distance(...) but for angle
     *
     * TODO: implement
     *
     * @param angle
     */
    void set_target_turn_angle(double angle) {
        this->move_target_turn_angle = angle;
    }

    double get_target_turn_angle() {
        return move_target_turn_angle;
    }

    int add_sensor(SensorInterface *sensor);

    std::vector<SensorInterface *> get_sensors();

    void set_turn_speed(double angle);

    double get_turn_speed() {
        return this-> move_angle;
    }

    void set_speed(double speed);

    double get_speed() {
        return this->move_speed;
    }

    void update();

    void draw_robot(cv::Mat image);
};

#endif //MR_CPP_CODE_ROBOT_H
