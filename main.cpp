#include <iostream>
#include "world.h"
#include "robot.h"
#include "sensor.h"
#include "constants.h"
#include "RobotOperator.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <unistd.h>

using chrono_clock = std::chrono::system_clock;
//using sec = std::chrono::duration<double>;
using chrono_ms = std::chrono::duration<double, std::milli>;


int main() {
    World world = World("world1.png");

    Robot r = Robot(std::string("r1"), 8, cv::Point2d(300.0, 70.0), 0.1 * M_PI, &world, M_PI/6, 100);
    Sensor s1 = Sensor(&world, &r, 0.20, 200);
    Sensor s2 = Sensor(&world, &r, -0.20, 200);
    Sensor s3 = Sensor(&world, &r, 0.45, 200);
    Sensor s4 = Sensor(&world, &r, -0.45, 200);
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
        cv::waitKey(1);

        const chrono_ms loop_duration = chrono_clock::now() - loop_start_chrono;
        const double target_ms = 1000 / TARGET_TPS / GAME_SPEED_MODIFIER;
        const double real_tps = 1000 / target_ms;
        if (loop_duration.count() < target_ms) {
            std::cout << "loop took " << loop_duration.count() << "ms" << std::endl;
            usleep((target_ms - loop_duration.count()) * 1000);
        } else {
            std::cout << "loop took " << loop_duration.count() << "ms; TPS: " << real_tps << "instead of target TPS: "
                      << TARGET_TPS << std::endl;
        }
    }
#pragma clang diagnostic pop


    return 0;
}
