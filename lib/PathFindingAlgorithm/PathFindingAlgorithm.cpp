//
// Created by markus on 29.05.21.
//

#include "PathFindingAlgorithm.h"
#include "../AStar/AStarDataStructures.h"

template <class T>
PathFindingAlgorithm<T>::PathFindingAlgorithm(std::string map_filename, double paddingRadius, unsigned int mapScaling, bool enableDiagonalTransition) {
    this->mapScaling = (int) mapScaling; // cv::Mat doesn't like unsigned integers -> casting to int -> still safe for calls from outside this class

    // load map, generate fastmap and padded map, convert padded map to mat again and show it
    cv::Mat grayscaleMap = cv::imread(map_filename, cv::IMREAD_GRAYSCALE);
    if ((grayscaleMap.cols == 0) || (grayscaleMap.rows == 0)) {
        throw std::invalid_argument("Error! Could not read the image file: '" + map_filename + "'");
    }

    FastMap paddedMap = padObstacles(FastMap(grayscaleMap), paddingRadius);
    this->scaledAndPaddedMap = paddedMap.getDownScaledMap(this->mapScaling);
    // This approach is a little faster but the result works not so good
//    this->scaledAndPaddedMap = padObstacles(FastMap(grayscaleMap).getDownScaledMap(this->mapScaling),paddingRadius / this->mapScaling);

    if (SHOW_WHATS_GOING_ON) {
        cv::Mat paddedImage = this->scaledAndPaddedMap.toCVMat();
        cv::imshow("padded image", paddedImage);
        cv::waitKey(1);
    }

    this->scaledMapBounds = this->scaledAndPaddedMap.getBounds();
    this->dataMap = PathFindingDataMap<T>(this->scaledMapBounds.x, this->scaledMapBounds.y);
    this->generateAdjacencyList(enableDiagonalTransition);
}

const int neighborsDiagonal[4][2] = {{-1, -1},
                                     {-1, 1},
                                     {1,  -1},
                                     {1,  1}};
const int neighborsDirect[4][2] = {{0,  -1},
                                   {-1, 0},
                                   {1,  0},
                                   {0,  1}};

template <class T>
void PathFindingAlgorithm<T>::generateAdjacencyList(bool enableDiagonalTransition) {
    // generate list. Skipping the outermost pixels to avoid a boundary-check-if. Those pixels cant be used anyways because our robot will have a size greater than 0.5
    for (int x = 1; x < this->scaledMapBounds.x - 1; ++x) {
        for (int y = 1; y < this->scaledMapBounds.y - 1; ++y) {
            // skip if current pixel is a wall
            if (this->scaledAndPaddedMap.getPixel(x, y)) continue;

            PathFindingDatapoint<T>* curDataPoint = this->dataMap.getPixel(x,y);
            // if neighbors are no boarder add them to the adjacency list
            if(enableDiagonalTransition) {
                for (auto &i : neighborsDiagonal) {
                    if (!this->scaledAndPaddedMap.getPixel(x + i[0], y + i[1]))
                        curDataPoint->adjacencyTargets.push_back(AdjacencyTarget({this->dataMap.XYToIndex(x + i[0], y + i[1]), 1.41421}));
                }
            }
            for (auto &i : neighborsDirect) {
                if (!this->scaledAndPaddedMap.getPixel(x + i[0], y + i[1]))
                    curDataPoint->adjacencyTargets.push_back(AdjacencyTarget({this->dataMap.XYToIndex(x + i[0], y + i[1]), 1.0}));
            }
        }
    }
}

template <class T>
unsigned int PathFindingAlgorithm<T>::getMapScaling() {
    return this->mapScaling;
}

template class PathFindingAlgorithm<AStarElement>;