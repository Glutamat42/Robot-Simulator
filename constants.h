//
// Created by markus on 05.04.21.
//

#ifndef MR_CPP_CODE_CONSTANTS_H
#define MR_CPP_CODE_CONSTANTS_H

const double CALCULATION_RESOLUTION = 0.1;
const double TARGET_TPS = 20;
const double GAME_SPEED_MODIFIER = 3; // this will modify the time calculated for one game loop step. A value of 2 will cause an acceleration of game speed by 2. This means 2 seconds will be calculated in one second, but it will look to the game like 2 seconds passed.
const std::string SIMULATOR_WINDOW_NAME = std::string("Robot simulator");

const int NOVELTY_THRESHOLD = 80;  // novelty threshold for novelty detection in robot operator

#endif //MR_CPP_CODE_CONSTANTS_H
