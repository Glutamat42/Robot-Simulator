//
// Created by markus on 25.04.21.
//

#ifndef MR_CPP_CODE_FASTASTAR_H
#define MR_CPP_CODE_FASTASTAR_H

#include <utility>

#include "../FastMap.h"
#include "AStarDataStructures.h"
#include "../PathFindingAlgorithm/PathFindingAlgorithm.h"
#include "../helpers.h"


class FastAStar: public PathFindingAlgorithm<AStarElement> {
private:
    /** This will run the A* loop
     *
     * @return points of shortest path starting from the target position. Coordinates are from the scaled map!
     */
    std::vector<AStarElement> runAStar();

    /** rescale the points to the original map resolution, resort array (first element -> startPos) and get only the points
     *
     */
    std::vector<cv::Point2d> aStarListToPointList(std::vector<AStarElement> path);

    /** update visualization of the map with the current f_cost
     *
     * @param image
     */
    void updateMap(cv::Mat* image);

    // <f_cost, heuristic, index>
    std::set<std::tuple<double, double, long>> aStarOpenListSet;

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
    FastAStar(std::string map_filename, double paddingRadius, unsigned int mapScaling = 1);

    /** as long as the map doesnt change the same instance can be used many times. Just set the new
     * parameters here and then start the loop
     *
     * @param bias Allows to over (or under) weight the heuristic. Increasing it will lead to faster results, but they won't be optimal anymore
     * @return true if everything went fine, false if start or target position are invalid (in wall)
     */
    bool setAStarParameters(cv::Point2i startPosition, cv::Point2i targetPosition, double bias = 1.0);

    std::vector<cv::Point2d> run() override;
};


#endif //MR_CPP_CODE_FASTASTAR_H
