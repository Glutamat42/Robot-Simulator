//
// Created by markus on 30.05.21.
//


#include "ValueIteration.h"
#include "../PathFindingAlgorithm/PathFindingAlgorithm.cpp"  // TODO: required?!?!
#include <chrono>

using chrono_clock = std::chrono::system_clock;
using chrono_ms = std::chrono::duration<double, std::milli>;

ValueIteration::ValueIteration(std::string map_filename, double paddingRadius, unsigned int mapScaling)
        : PathFindingAlgorithm<ValueIterationElement>(map_filename, paddingRadius, mapScaling, false) {
    // TODO: crashes if bounds/mapScaling is not an integer or mapScaling is 1
    if (this->scaledMapBounds.x / this->mapScaling > 30) {
        std::cout << "Scaled map is still quite large. Decrease map size if the performance is bad or you have visualization problems" << std::endl;
    }
}  // TODO: adjaceny list isnt used here at all

void ValueIteration::setParams(cv::Point2i startPos,
                               cv::Point2i winPos,
                               double winPosReward,
                               cv::Point2i loosePos,
                               double loosePosReward,
                               double defaultReward,
                               double gamma,
                               double probabilityOfDesiredMovement) {
    this->startPos = startPos/this->mapScaling;
    this->winPos = winPos/this->mapScaling;
    this->winPosReward = winPosReward;
    this->loosePos = loosePos/this->mapScaling;
    this->loosePosReward = loosePosReward;
    this->defaultReward = defaultReward;
    this->gamma = gamma;
    this->probabilityOfDesiredMovement = probabilityOfDesiredMovement;
}

[[noreturn]] std::vector<cv::Point2d> ValueIteration::run() {
    // for all 4 possible neighbors: left(x,y), straight(x,y), right(x,y) where straight == desired movement
    static const int neighborsDirect[4][3][2] = {{{0,  1}, {1,  0},  {0,  -1}},
                                                {{1,  0},  {0,  -1},  {-1, 0}},
                                                {{0,  -1},  {-1, 0},  {0,  1}},
                                                {{-1, 0},  {0,  1}, {1,  0}}};

    std::random_device rd;
    std::mt19937 mt(rd());
    std::uniform_real_distribution<double> dist(0.0, 1.0);

    std::vector<cv::Point2i> validPositions;

    // init cell values
    for (int x = 0; x < this->scaledMapBounds.x; ++x) {
        for (int y = 0; y < this->scaledMapBounds.y; ++y) {
            if (this->scaledAndPaddedMap.getPixel(x, y)) continue; // is wall: skip

            PathFindingDatapoint<ValueIterationElement> *currentDataPoint = this->dataMap.getPixel(x, y);
            cv::Point2i curPos = cv::Point2i(x, y);

            currentDataPoint->element.V[0] = dist(mt); // set random V
            currentDataPoint->element.validField = true; // mark as valid field (is not a wall)

            validPositions.push_back(curPos); // add to valid positions list for faster and easier future access

            // set cell reward
            if (curPos == this->winPos) {
                currentDataPoint->element.cellReward = this->winPosReward;
            } else if (curPos == this->loosePos) {
                currentDataPoint->element.cellReward = this->loosePosReward;
            } else {
                currentDataPoint->element.cellReward = this->defaultReward;
            }
        }
    }

    unsigned int iterations = 0;
    auto loop_start_chrono = chrono_clock::now();
    while (true) { // TODO: detect convergence and stop if changes are negligible
        for (cv::Point2i p: validPositions) {
            PathFindingDatapoint<ValueIterationElement> *currentDataPoint = this->dataMap.getPixel(p.x, p.y);

            double bestActionReward = -DBL_MAX;
            int bestActionNeighborIndex = -1;

            for (int i = 0; i < 4; ++i) {
                auto &neighborOffset = neighborsDirect[i];
                PathFindingDatapoint<ValueIterationElement> *neighborLeft = this->dataMap.getPixel(p.x + neighborOffset[0][0],
                                                                                                   p.y + neighborOffset[0][1]);
                PathFindingDatapoint<ValueIterationElement> *neighborTarget = this->dataMap.getPixel(p.x + neighborOffset[1][0],
                                                                                                     p.y + neighborOffset[1][1]);
                PathFindingDatapoint<ValueIterationElement> *neighborRight = this->dataMap.getPixel(p.x + neighborOffset[2][0],
                                                                                                    p.y + neighborOffset[2][1]);
                if (!neighborTarget->element.validField) continue; // is wall: skip

                // TODO: will probably fail if the current field is at the map border (eg (0,x))
                double actionRewardMain = this->probabilityOfDesiredMovement * neighborTarget->element.V[iterations];
                double actionRewardLeft = (1 - this->probabilityOfDesiredMovement) / 2 *  (neighborLeft->element.validField ? neighborLeft->element.V[iterations] : currentDataPoint->element.V[iterations]);
                double actionRewardRight = (1 - this->probabilityOfDesiredMovement) / 2 * (neighborRight->element.validField ? neighborRight->element.V[iterations] : currentDataPoint->element.V[iterations]);

                double actionReward = actionRewardLeft + actionRewardMain + actionRewardRight;

                if (actionReward > bestActionReward) {
                    bestActionReward = actionReward;
                    bestActionNeighborIndex = i;
                }
            }

            currentDataPoint->element.V.push_back(currentDataPoint->element.cellReward + this->gamma * bestActionReward);
            currentDataPoint->element.bestDirection.push_back(bestActionNeighborIndex);
        }

        this->updateMap();

        iterations++;
        const chrono_ms loop_duration = chrono_clock::now() - loop_start_chrono;
        loop_start_chrono = chrono_clock::now();
        std::cout << "completed " << iterations << " iteration(s), last loop took " << loop_duration.count() << "ms" << std::endl;
    }

    std::cout << "NOT IMPLEMENTED: return calculated path";
    return std::vector<cv::Point2d>();
}

void ValueIteration::updateMap() {
    Visualize visualize(this->scaledMapBounds);

    for (int x = 0; x < this->scaledMapBounds.x; ++x) {
        for (int y = 0; y < this->scaledMapBounds.y; ++y) {
            PathFindingDatapoint<ValueIterationElement> *currentDataPoint = this->dataMap.getPixel(x, y);
            MapField mapField;

            if (currentDataPoint->element.validField) {
                // valid field
                // set text, bg color
                mapField.text = std::to_string(currentDataPoint->element.V.back());
                mapField.color_bg = Colors::white;

                // bg color for win/loose/start pos
                if (cv::Point2i(x,y) == this->winPos) {
                    mapField.color_bg = Colors::green;
                } else if (cv::Point2i(x,y) == this->loosePos) {
                    mapField.color_bg = Colors::red;
                } else if (cv::Point2i(x,y) == this->startPos) {
                    mapField.color_bg = Colors::gray;
                }

                // direction indicator
                switch (currentDataPoint->element.bestDirection.back()) {
                    case 0:
                        mapField.setColorRight(Colors::blue);
                        break;
                    case 1:
                        mapField.setColorTop(Colors::blue);
                        break;
                    case 2:
                        mapField.setColorLeft(Colors::blue);
                        break;
                    case 3:
                        mapField.setColorBottom(Colors::blue);
                        break;
                    case -1:
                        std::cout << "direction is -1. this is the initial value." << std::endl;
                        break;
                    default:
                        break;
                }
            } else {
                // invalid field -> wall -> set bg to black
                mapField.color_bg = Colors::black;
            }

            visualize.setCell(cv::Point2i(x,y), mapField);
        }
    }

    if(SHOW_WHATS_GOING_ON) visualize.show(true);
}
