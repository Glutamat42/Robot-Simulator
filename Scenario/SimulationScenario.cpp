//
// Created by markus on 19.04.21.
//

#include <chrono>
#include <unistd.h>
#include "SimulationScenario.h"

using chrono_clock = std::chrono::system_clock;
using chrono_ms = std::chrono::duration<double, std::milli>;

void SimulationScenario::init(bool pause) {
    for (SensorInterface* sensor : sensors) {
        sensor->update_sensor_data(false);
    }

    world->show_map();
    if (pause) {
        cv::waitKey();
    } else {
        cv::waitKey(1);
    }
}

void SimulationScenario::startLoop() {
    // Begin game loop stuff
    const double TARGET_TPS = GAME_TPS * GAME_SPEED_MODIFIER; // This is the real tps we want to reach and depends on the game speed modifier. For all ingame calculations just GAME_TPS is used
    const double target_ms = 1000 / GAME_TPS / GAME_SPEED_MODIFIER;

    // log some configs
    std::cout << "Game TPS: " << GAME_TPS << " Game speed modifier: " << GAME_SPEED_MODIFIER << " resulting in target tps of: " << TARGET_TPS << std::endl;

    // perf measurement variables
    double perf_measurement_duration_counter = 0; // will sum up the calculation time used in one second and then be reset
    short perf_measurement_iterations_counter = 0;

#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
    while (true) {
        const auto loop_start_chrono = chrono_clock::now();

        for (RobotOperator* robotOperator: robotOperators) {
            robotOperator->update();
        }

        for (Robot* robot: world->get_robots()) {
            robot->update();
        }

        // sensor data has to be updated after everything else
        for (SensorInterface* sensor : sensors) {
            sensor->update_sensor_data(false);
        }

        world->show_map();

        // simple pause function
        char keyPressed = cv::waitKey(1);
        if (keyPressed == (char) 32) {
            bool key_was_released = false;
            std::cout << "Paused" << std::endl;
            while (true) {
                if (!key_was_released && (char) cv::waitKey(1) != (char) 32) key_was_released = true;
                if (key_was_released && ((char) cv::waitKey(1) == (char) 32)) {
                    std::cout << "Continue" << std::endl;
                    break;
                }
            }
        } else if (keyPressed == (char) 27) {
            std::cout << "Pressed ESC -> exit" << std::endl;
            exit(0);
        }

        const chrono_ms loop_duration = chrono_clock::now() - loop_start_chrono;

        // sleep if execution was faster than it should be based on tps and game speed
        if (loop_duration.count() < target_ms) {
//            std::cout << "loop took " << loop_duration.count() << "ms" << std::endl;
            usleep((target_ms - loop_duration.count()) * 1000);
        }

        // performance measurement
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
}

SimulationScenario::~SimulationScenario() {
    for (auto robot : this->robots) {
        delete robot;
    }
    for (auto robotOperator : this->robotOperators) {
        delete robotOperator;
    }
    for (auto sensor : this->sensors) {
        delete sensor;
    }

}
