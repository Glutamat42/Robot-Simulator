//
// Created by markus on 30.05.21.
//

#ifndef MR_CPP_CODE_VALUEITERATION_H
#define MR_CPP_CODE_VALUEITERATION_H


#include "../PathFindingAlgorithm/PathFindingAlgorithm.h"
#include "ValueIterationDatastructures.h"
#include <random>
#include <iostream>
#include "Visualize/Visualize.h"

class ValueIteration : public PathFindingAlgorithm<ValueIterationElement> {
private:
    cv::Point2i startPos;
    cv::Point2i winPos;
    double winPosReward;
    cv::Point2i loosePos;
    double loosePosReward;
    double defaultReward;
    double gamma;
    double probabilityOfDesiredMovement;

    /** update visualization of the map with the current f_cost
     *
     * @param image
     */
    void updateMap();
public:
    ValueIteration(std::string map_filename, double paddingRadius, unsigned int mapScaling = 1);

    [[noreturn]] std::vector<cv::Point2d> run() override;

    /** Set parameters specific to Value Iteration.
     * Remember Positions might be scaled down (depending on mapScaling)
     *
     * @param probabilityOfDesiredMovement value between 0 and 1. (1 - x)/2 for 90° left and right
     */
    void setParams(cv::Point2i startPos, cv::Point2i winPos, double winPosReward, cv::Point2i loosePos, double loosePosReward, double defaultReward, double gamma, double probabilityOfDesiredMovement = 0.8);
};


#endif //MR_CPP_CODE_VALUEITERATION_H
