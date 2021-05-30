//
// Created by markus on 29.05.21.
//

#ifndef MR_CPP_CODE_PATHFINDINGALGORITHM_H
#define MR_CPP_CODE_PATHFINDINGALGORITHM_H


#include <string>
#include "../FastMap.h"
#include "PathFindingDataMap.h"
#include <stdexcept>
#include <opencv2/core/mat.hpp>
#include "../../Simulator/constants.h"
#include "../helpers.h"
#include "PathFindingDataStructures.h"

template <class T>

class PathFindingAlgorithm {
protected:
    int mapScaling;  // scaled map size = 1/MAP_SCALING
    FastMap scaledAndPaddedMap;
    cv::Point2i scaledMapBounds;

    void generateAdjacencyList(bool enableDiagonalTransition);

    std::vector<cv::Point2d> path;

    PathFindingDataMap<T> dataMap;

public:
    /**
     *
     * @param map_filename
     * @param paddingRadius
     * @param mapScaling
     * @param enableDiagonalTransition if enabled diagonal transitions between points will be added to adjacency list with a weight of around 1.41, otherwise only direct neighbors will be added
     */
    PathFindingAlgorithm(std::string map_filename, double paddingRadius, unsigned int mapScaling = 1, bool enableDiagonalTransition = true);
    virtual void run() = 0;

    std::vector<cv::Point2d> getPath();

    /** get the map scaling factor */
    unsigned int getMapScaling();
};


#endif //MR_CPP_CODE_PATHFINDINGALGORITHM_H
