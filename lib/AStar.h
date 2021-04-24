//
// Created by markus on 23.04.21.
//

#ifndef MR_CPP_CODE_ASTAR_H
#define MR_CPP_CODE_ASTAR_H


#include "FastMap.h"

const int MAP_SCALING = 6;  // scaled map size = 1/MAP_SCALING

struct AdjacencyTarget {
    cv::Point2i position;
    double weight;
};
struct AdjacencyRow {
    cv::Point2i from;
    std::vector<AdjacencyTarget> to;
};
struct AdjacencyList {
    std::vector<AdjacencyRow> list;
    std::vector<int> searchIndex;
    int searchIndexXMultiplier;
};
struct AStarElement {
    cv::Point2i position;
    cv::Point2i prev;
    double heuristic;
    double distance;
    double f_cost;
    bool startNode = false;
};

class AStar {
private:
//    FastMap map;
    FastMap scaledAndPaddedMap;

    // adjacency list
    AdjacencyList adjacencyList;
    AdjacencyList static generateAdjacencyList(FastMap map);

    // AStar
    cv::Point2i startPos;
    cv::Point2i targetPos;
    double heuristicBias;
    std::vector<AStarElement> openList;
    std::vector<AStarElement> closedList;

public:
    /**
     *
     * @param map_filename
     * @param paddingRadius should at least cover the size of the robot. Maybe add some additional padding
     */
    AStar(std::string map_filename, double paddingRadius);

    /** as long as the map doesnt change the same instance can be used many times. Just set the new
     * parameters here and then start the loop
     *
     * @param bias Allows to over (or under) weight the heuristic. Increasing it will lead to faster results, but they won't be optimal anymore
     */
    void setAStarParameters(cv::Point2i startPosition, cv::Point2i targetPosition, double bias = 1.0);

    /** This will run the A* loop
     *
     * @return points of shortest path starting from the target position. Coordinates are from the scaled map!
     */
    std::vector<AStarElement> runAStar();

    /** rescale the points to the original map resolution, resort array (first element -> startPos) and get only the points
     *
     */
     std::vector<cv::Point2d> static aStarListToPointList(std::vector<AStarElement> path);
};


#endif //MR_CPP_CODE_ASTAR_H
