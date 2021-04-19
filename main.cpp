#include <iostream>
#include "world.h"
#include "interactive_objects/robot.h"
#include "interactive_objects/DistanceSensor.h"
#include "constants.h"
#include "RobotOperator.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <unistd.h>
#include <random>

using chrono_clock = std::chrono::system_clock;
using chrono_ms = std::chrono::duration<double, std::milli>;


int main() {
    srand(time(NULL));

    World world = World("world1.png");


//    // init main test robot
//    Robot r = Robot(std::string("r1"), 8, cv::Point2d(320.0, 50.0), 1.18 * M_PI, &world, M_PI / 3, 10);
//    DistanceSensor s1 = DistanceSensor(&world, &r, 0.20 * M_PI, 200);
//    DistanceSensor s2 = DistanceSensor(&world, &r, -0.20 * M_PI, 200);
//    DistanceSensor s3 = DistanceSensor(&world, &r, 0.45 * M_PI, 200);
//    DistanceSensor s4 = DistanceSensor(&world, &r, -0.45 * M_PI, 200);
//    r.add_sensor(&s1);
//    r.add_sensor(&s2);
//    r.add_sensor(&s3);
//    r.add_sensor(&s4);
//    world.add_object(&r);
//
//    RobotOperator ro = RobotOperator(&r);
//
//    Robot r2 = Robot(std::string("r2"), 8, cv::Point2d(280.0, 50.0), 0.2 * M_PI, &world, M_PI / 3, 10);
//    DistanceSensor r2s1 = DistanceSensor(&world, &r2, 0.2*M_PI, 200);
//    r2.add_sensor(&r2s1);
//    DistanceSensor r2s2 = DistanceSensor(&world, &r2, -0.2*M_PI, 200);
//    r2.add_sensor(&r2s2);
//    world.add_object(&r2);
//    RobotOperator ro2 = RobotOperator(&r2);
//
//
//    world.show_map();
//    cv::waitKey();
//    r.update();
//    r2.update();
//    world.show_map();
//    cv::waitKey();
//    sleep(5);
//    cv::waitKey();
    // END: init main test robot

    // init robot army
    std::vector<Robot*> robots;
    std::vector<RobotOperator*> robotOperators;
    std::vector<DistanceSensor*> sensors;

    for (int i = 0; i < 25; ++i) {
        Robot* robot = new Robot(std::string("robot_army_" + std::to_string(i)), 8, &world, M_PI / 3, 200);
        DistanceSensor* sensor1 = new DistanceSensor(&world, robot, 0.2*M_PI, 199);
        robot->add_sensor(sensor1);
        DistanceSensor* sensor2 = new DistanceSensor(&world, robot, -0.2*M_PI, 198);
        robot->add_sensor(sensor2);
        world.add_object(robot);
        RobotOperator* robotOperator = new RobotOperator(robot);
        robot->update();

        sensors.push_back(sensor1);
        sensors.push_back(sensor2);
        robots.push_back(robot);
        robotOperators.push_back(robotOperator);
    }
    // END: init robot army


    world.show_map();
    cv::waitKey();


    // Begin game loop stuff
    const double TARGET_TPS = GAME_TPS * GAME_SPEED_MODIFIER; // This is the real tps we want to reach and depends on the game speed modifier. For all ingame calculations just GAME_TPS is used
    const double target_ms = 1000 / GAME_TPS / GAME_SPEED_MODIFIER;
    const double real_tps = 1000 / target_ms;

    // log some configs
    std::cout << "Game TPS: " << GAME_TPS << " Game speed modifier: " << GAME_SPEED_MODIFIER << " resulting in target tps of: " << TARGET_TPS << std::endl;

    // perf measurement variables
    double perf_measurement_duration_counter = 0; // will sum up the calculation time used in one second and then be reset
    short perf_measurement_iterations_counter = 0;

#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
    while (true) {
        const auto loop_start_chrono = chrono_clock::now();

//        ro.update();
//        ro2.update();
        for (RobotOperator* robotOperator: robotOperators) {
            robotOperator->update();
        }

        for (Robot* robot: world.get_robots()) {
            robot->update();
        }
        world.show_map();

        // simple pause function
        if ((char) cv::waitKey(1) == (char) 32) {
            bool key_was_released = false;
            std::cout << "Paused" << std::endl;
            while (true) {
                if (!key_was_released && (char) cv::waitKey(1) != (char) 32) key_was_released = true;
                if (key_was_released && ((char) cv::waitKey(1) == (char) 32)) {
                    std::cout << "Continue" << std::endl;
                    break;
                }
            }
        }


        const chrono_ms loop_duration = chrono_clock::now() - loop_start_chrono;
        if (loop_duration.count() < target_ms) {
//            std::cout << "loop took " << loop_duration.count() << "ms" << std::endl;
            usleep((target_ms - loop_duration.count()) * 1000);
        } else {
            std::cout << "loop took " << loop_duration.count() << "ms; TPS: " << real_tps << "instead of target TPS: "
                      << TARGET_TPS << std::endl;
        }
        perf_measurement_duration_counter += loop_duration.count();
        perf_measurement_iterations_counter++;
        if (perf_measurement_iterations_counter == GAME_TPS * GAME_SPEED_MODIFIER) {
            int percentage = perf_measurement_duration_counter / 10; // / 1000 * 100
            std::cout << "required " << (int)perf_measurement_duration_counter << "ms (" << percentage << "%)" << std::endl;
            perf_measurement_duration_counter = 0;
            perf_measurement_iterations_counter = 0;
        }
    }
#pragma clang diagnostic pop


    return 0;
}
