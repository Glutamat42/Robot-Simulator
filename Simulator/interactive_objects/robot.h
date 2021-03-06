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
#include "../interfaces/SensorInterface.h"
#include "interfaces/CollidableCircle.h"


class Robot : public CollidableCircle, public RobotControlInterface {
protected:
    std::string name;
    double orientation;
    double max_angle; // max angle per second
    double max_speed; // max speed in px/s
    double move_angle = 0;
    double move_speed = 0;
    double move_target_distance = 0; // will set_target_move_distance for this amount of pixels and then stop
    double move_target_turn_angle = 0; // will set_target_turn_angle for this angle and then stop turning
    std::vector<SensorInterface *> sensors;

    double last_tick_movement_distance = 0;
    double last_tick_movement_angle = 0;

    // visualization options
    cv::Scalar robotCircleColor = CV_RGB(255, 0, 0);
    bool hideDirectionIndicator = false;
    int customDrawRadius = 0;

    explicit Robot(World *world);
public:

    Robot(std::string name, int radius, cv::Point2d start_pos, double start_orientation, World *world,
          double max_angle = M_PI / 6, double max_speed = 50.0);

    Robot(std::string name, int radius, World *world, double max_angle = M_PI / 6, double max_speed = 50.0);

    void handleCollision(CollidableObject *object) override;

    void update();

    void draw_robot(cv::Mat image);


    /** specify distance until robot should stop moving. It'll set_target_move_distance with the specified speed (set_speed)
     * If this function is not called the robot will set_target_move_distance until stopped by manually setting move_speed to 0
     * or setting a distance limit by this function
     *
     * @param pixel
     */
    void set_target_move_distance(double pixel) override {
        this->move_target_distance = pixel;
    };

    double get_target_move_distance() override {
        return move_target_distance;
    }

    void setDrawOptions(cv::Scalar color, bool hideDirectionIndicator = false, int customRadius = 0);

    std::string get_name() override;

    double get_radius();

    double get_orientation();

    /** same as set_target_move_distance(...) but for angle
     *
     * @param angle
     */
    void set_target_turn_angle(double angle) override {
        this->move_target_turn_angle = angle;
    }

    double get_target_turn_angle() override {
        return move_target_turn_angle;
    }

    int add_sensor(SensorInterface *sensor);

    std::vector<SensorInterface *> get_sensors() override;

    /** clear the list of sensors
     *
     * @param deletePointers if true also delete the objects in the sensors list
     */
    void clearSensorsList(bool deletePointers = false);

    void set_turn_speed(double angle) override;

    double get_turn_speed() override {
        return this->move_angle;
    }

    void set_speed(double speed) override;

    double get_speed() override {
        return this->move_speed;
    }

    double get_last_tick_movement_distance() override { return this->last_tick_movement_distance; }

    double get_last_tick_movement_angle() override { return this->last_tick_movement_angle; }

    double get_max_turn_rate() override;

    double get_max_move_speed() override;
};

#endif //MR_CPP_CODE_ROBOT_H
