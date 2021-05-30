//
// Created by markus on 25.04.21.
//

#ifndef MR_CPP_CODE_ASTARDATASTRUCTURES_H
#define MR_CPP_CODE_ASTARDATASTRUCTURES_H

#include "../ValueIteration/PathFindingDataStructures.h"


struct AStarElement : public PathFindingElement {
    AStarElement(long prevIndex = 0,
                 double heuristic = 0,
                 double distance = 0,
                 double f_cost = 0,
                 bool startNode=false,
                 bool isOpenList=false,
                 bool isClosedList=false,
                 cv::Point2i position=cv::Point2i (0,0)){
        this->prevIndex = prevIndex;
        this->heuristic = heuristic;
        this->distance = distance;
        this->f_cost = f_cost;
        this->startNode = startNode;
        this->isOpenList = isOpenList;
        this->isClosedList = isClosedList;
        this->position = position;
    };
    long prevIndex=0;
    double heuristic=0;
    double distance=0;
    double f_cost=0;
    bool startNode = false;
    bool isOpenList = false;
    bool isClosedList = false;
    cv::Point2i position = cv::Point2i(0,0);  // only used after A* pathfinding completed

//    void test() {};
};


#endif //MR_CPP_CODE_ASTARDATASTRUCTURES_H
