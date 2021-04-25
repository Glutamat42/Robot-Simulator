//
// Created by markus on 25.04.21.
//

#ifndef MR_CPP_CODE_FASTASTAR_H
#define MR_CPP_CODE_FASTASTAR_H

#include <utility>

#include "FastMap.h"

const int MAP_SCALING = 1;  // scaled map size = 1/MAP_SCALING

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


class AStarMap {
private:
    std::vector<AStarDatapoint> aStarList;
    cv::Point2i bounds;
public:
    AStarMap() : AStarMap(0,0) {}

    AStarMap(int x, int y) {
        this->bounds = cv::Point2i(x, y);
        this->aStarList = std::vector<AStarDatapoint>(this->bounds.x * this->bounds.y, AStarDatapoint());
    }

    long XYToIndex(int x, int y) {
        return this->bounds.y * y + x;
    }

    cv::Point2i IndexToXY(long index) {
        int y = index / this->bounds.y;
        int x = index % this->bounds.y;
        return cv::Point2i(x,y);
    }

    void setPixelAstarElement(int x, int y, AStarElement element) {
        int index = this->bounds.y * y + x;
        if (index >= this->bounds.y * this->bounds.x) throw std::invalid_argument("index is out of map");
        aStarList[this->bounds.y * y + x].aStarElement = std::move(element);
    }

    void resetAStarData() {
        for (int i = 0; i < this->aStarList.size(); ++i) {
            AStarDatapoint* datapoint = &this->aStarList[i];
            datapoint->aStarElement = AStarElement();
        }
    }

    AStarDatapoint* getPixel(int x, int y) {
        long index = this->bounds.y * y + x;
        return this->getPixel(index);
    }

    AStarDatapoint* getPixel(long index) {
        if (index >= this->bounds.y * this->bounds.x) throw std::invalid_argument("index is out of map");
        return &aStarList[index];
    }

    cv::Point2i getBounds() {
        return this->bounds;
    }
};


class FastAStar {
private:
    FastMap scaledAndPaddedMap;
    cv::Point2i scaledMapBounds;

    AStarMap aStarMap;
    std::vector<long> aStarListOpen;
    // <f_cost, index>
    std::map<std::tuple<double, double, long>, long> aStarMapOpen;

    // adjacency list
    void generateAdjacencyList();

    // AStar
    cv::Point2i startPos;
    cv::Point2i targetPos;
    long targetIndex;
    double heuristicBias;
public:
    /**
     *
     * @param map_filename
     * @param paddingRadius should at least cover the size of the robot. Maybe add some additional padding
     */
    FastAStar(std::string map_filename, double paddingRadius);

    /** as long as the map doesnt change the same instance can be used many times. Just set the new
     * parameters here and then start the loop
     *
     * @param bias Allows to over (or under) weight the heuristic. Increasing it will lead to faster results, but they won't be optimal anymore
     * @return true if everything went fine, false if start or target position are invalid (in wall)
     */
    bool setAStarParameters(cv::Point2i startPosition, cv::Point2i targetPosition, double bias = 1.0);

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


#endif //MR_CPP_CODE_FASTASTAR_H
