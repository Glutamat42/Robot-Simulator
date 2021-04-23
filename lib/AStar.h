//
// Created by markus on 23.04.21.
//

#ifndef MR_CPP_CODE_ASTAR_H
#define MR_CPP_CODE_ASTAR_H


#include "FastMap.h"

const int MAP_SCALING = 4;  // scaled map size = 1/MAP_SCALING
const bool SHOW_WHATS_GOING_ON = true;

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
    std::vector<AStarElement> openList;
    std::vector<AStarElement> closedList;

public:
    AStar(std::string map_filename, double paddingRadius);

    /** as long as the map doesnt change the same instance can be used many times. Just set the new
     * parameters here and then start the loop
     */
    void setAStarParameters(cv::Point2i startPos, cv::Point2i targetPos);

    /** This will run the A* loop
     *
     * @return points of shortest path starting from the target position. Coordinates are from the scaled map!
     */
    std::vector<AStarElement> runAStar();

    /** rescale the points to the original map resolution, resort array (first element -> startPos) and get only the points
     *
     */
     std::vector<cv::Point2i> static astarListToPointList(std::vector<AStarElement> path);
};


#endif //MR_CPP_CODE_ASTAR_H
