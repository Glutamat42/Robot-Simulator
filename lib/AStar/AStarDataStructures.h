//
// Created by markus on 25.04.21.
//

#ifndef MR_CPP_CODE_ASTARDATASTRUCTURES_H
#define MR_CPP_CODE_ASTARDATASTRUCTURES_H

struct AdjacencyTarget {
    long index;
    double weight;
};

struct AStarElement {
    long prevIndex=0;
    double heuristic=0;
    double distance=0;
    double f_cost=0;
    bool startNode = false;
    bool isOpenList = false;
    bool isClosedList = false;
    cv::Point2i position = cv::Point2i(0,0);  // only used after A* pathfinding completed
};
struct AStarDatapoint {
    std::vector<AdjacencyTarget> adjacencyTargets;

    AStarElement aStarElement;
};

#endif //MR_CPP_CODE_ASTARDATASTRUCTURES_H
