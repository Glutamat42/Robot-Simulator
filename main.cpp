#include <iostream>
#include "world.h"
#include "robot.h"
#include "DistanceSensor.h"
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

    Robot r = Robot(std::string("r1"), 8, cv::Point2d(300.0, 70.0), 0.1 * M_PI, &world, M_PI / 3, 200);
    DistanceSensor s1 = DistanceSensor(&world, &r, 0.20 * M_PI, 200);
    DistanceSensor s2 = DistanceSensor(&world, &r, -0.20 * M_PI, 200);
    DistanceSensor s3 = DistanceSensor(&world, &r, 0.45 * M_PI, 200);
    DistanceSensor s4 = DistanceSensor(&world, &r, -0.45 * M_PI, 200);
    r.add_sensor(&s1);
    r.add_sensor(&s2);
    r.add_sensor(&s3);
    r.add_sensor(&s4);
    world.add_robot(&r);

    RobotOperator ro = RobotOperator(&r);

    r.update();
    world.show_map();
    cv::waitKey();


#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
    while (true) {
        const auto loop_start_chrono = chrono_clock::now();

        ro.update();
        r.update();
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
        const double target_ms = 1000 / TARGET_TPS / GAME_SPEED_MODIFIER;
        const double real_tps = 1000 / target_ms;
        if (loop_duration.count() < target_ms) {
//            std::cout << "loop took " << loop_duration.count() << "ms" << std::endl;
            usleep((target_ms - loop_duration.count()) * 1000);
        } else {
            std::cout << "loop took " << loop_duration.count() << "ms; TPS: " << real_tps << "instead of target TPS: "
                      << TARGET_TPS << std::endl;
        }
    }
#pragma clang diagnostic pop


    return 0;
}
