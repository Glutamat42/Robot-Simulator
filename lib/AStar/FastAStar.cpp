//
// Created by markus on 23.04.21.
//

#include "FastAStar.h"
#include "../ValueIteration/PathFindingAlgorithm.cpp"


FastAStar::FastAStar(std::string map_filename, double paddingRadius, unsigned int mapScaling) : PathFindingAlgorithm<AStarElement>(map_filename, paddingRadius, mapScaling) {

}

const int neighbors[8][2] = {{-1, -1},
                             {-1, 1},
                             {1,  -1},
                             {1,  1},
                             {0,  -1},
                             {-1, 0},
                             {1,  0},
                             {0,  1}};

bool FastAStar::setAStarParameters(cv::Point2i startPosition, cv::Point2i targetPosition, double bias) {
    cv::Point2i _startPos = startPosition / this->mapScaling;
    cv::Point2i _targetPos = targetPosition / this->mapScaling;

    if (this->scaledAndPaddedMap.getPixel(_startPos.x, _startPos.y) || this->scaledAndPaddedMap.getPixel(_targetPos.x, _targetPos.y)) {
        std::cout << "invalid start or target position" << std::endl;
        return false;
    }

    this->startPos = _startPos;
    this->targetPos = _targetPos;
    this->targetIndex = this->dataMap.XYToIndex(this->targetPos.x, this->targetPos.y);
    this->heuristicBias = bias;

    if (bias != 1.0) std::cout << "Using heuristic bias of " << this->heuristicBias << std::endl;

//    this->dataMap.resetData();  // TODO: probably not required
    this->aStarOpenListSet.clear();

    // add startPos to openList
    this->dataMap.setPixelElement(
            this->startPos.x,
            this->startPos.y,
            AStarElement({this->dataMap.XYToIndex(this->startPos.x, this->startPos.y), 0, 0, 0, true, true, false, cv::Point2i()})
    );
    this->aStarOpenListSet.insert(std::tuple<double, double, long>(std::tuple(0.0, 0.0, this->dataMap.XYToIndex(this->startPos.x, this->startPos.y))));

    return true;
}

void FastAStar::updateMap(cv::Mat* image) {
    const int weightColorDivider = (int) sqrt(this->scaledMapBounds.x * this->scaledMapBounds.x + this->scaledMapBounds.y * this->scaledMapBounds.y);  // simply looks not bad on my example

    // show current open and closed list on map
    for (int x = 0; x < this->scaledMapBounds.x; ++x) {
        for (int y = 0; y < this->scaledMapBounds.y; ++y) {
            PathFindingDatapoint<AStarElement>* aStarDatapoint = this->dataMap.getPixel(x,y);
            if (aStarDatapoint->element.isClosedList)
                image->at<cv::Vec3b>(cv::Point2i(x,y)) = cv::Vec3b(0, 0, 255 * (int) aStarDatapoint->element.f_cost / weightColorDivider);
            else if (aStarDatapoint->element.isOpenList)
                image->at<cv::Vec3b>(cv::Point2i(x,y)) = cv::Vec3b(0, 255, 0);
        }
    }
    image->at<cv::Vec3b>(this->startPos) = cv::Vec3b(255, 0, 0);
    image->at<cv::Vec3b>(this->targetPos) = cv::Vec3b(255, 255, 255);
    cv::imshow("A*", *image);
    cv::waitKey(1);
}

std::vector<AStarElement> FastAStar::runAStar() {
    cv::Mat image;
    int loopCounter = 0;
    AStarElement targetNodeAStarElement;

    if (SHOW_WHATS_GOING_ON) {
        cv::cvtColor(this->scaledAndPaddedMap.toCVMat(), image, cv::COLOR_GRAY2RGB);
    }

    while (true) {
        if (SHOW_WHATS_GOING_ON && loopCounter % 25000 == 0) {
            this->updateMap(&image);
        }
        loopCounter++;


        long lowestFcostIndex = std::get<2>(*this->aStarOpenListSet.begin());
        this->aStarOpenListSet.erase(this->aStarOpenListSet.begin());
        PathFindingDatapoint<AStarElement>* currentNode = this->dataMap.getPixel(lowestFcostIndex);

        currentNode->element.isOpenList = false;
        currentNode->element.isClosedList = true;


        // check if the currentNode node is the target
        if (lowestFcostIndex == this->targetIndex) {
            targetNodeAStarElement = currentNode->element;
            break;
        };


        // iterate through neighbors
        for (AdjacencyTarget &curNeighborAdjacencyEntry : currentNode->adjacencyTargets) {
            AStarElement* currentNeighborElement = &this->dataMap.getPixel(curNeighborAdjacencyEntry.index)->element;
            // check if neighbor is closed -> if yes skip this node (nothing to do for this node)
            if (currentNeighborElement->isClosedList) continue;

            // calculate distance from start over current node to current neighbor
            double newDistance = currentNode->element.distance + curNeighborAdjacencyEntry.weight;

            // search for currentNeighbor in openList
            bool isInOpenList = currentNeighborElement->isOpenList;

            // if entry found in open list, check if newDistance is greater than the previous found path to neighbor
            if (isInOpenList && newDistance >= currentNeighborElement->distance) {
                // old way is better or equal -> skip
                continue;
            }

            // if not yet in openlist: calculate heuristic (h_cost) and mark as in openList
            if (!isInOpenList) {
                cv::Point2i heuristic_distance = this->dataMap.IndexToXY(curNeighborAdjacencyEntry.index) - this->targetPos;
                currentNeighborElement->heuristic = sqrt(heuristic_distance.x * heuristic_distance.x + heuristic_distance.y * heuristic_distance.y);
                currentNeighborElement->isOpenList = true;
            } else {
                // if already in openList: remove it from the openList set (and only openlist set, not the stored heuristic etc.)
                this->aStarOpenListSet.erase(std::tuple<double, double, long>(currentNeighborElement->f_cost, currentNeighborElement->heuristic, curNeighborAdjacencyEntry.index));
            }

            // calculate and set distance (g_cost) and f_cost
            currentNeighborElement->distance = newDistance;
            currentNeighborElement->f_cost = currentNeighborElement->distance + currentNeighborElement->heuristic * this->heuristicBias;
            currentNeighborElement->prevIndex = lowestFcostIndex;

            // add (updated) entry to openList set
            this->aStarOpenListSet.insert(std::tuple<double, double, long>(currentNeighborElement->f_cost, currentNeighborElement->heuristic, curNeighborAdjacencyEntry.index));
        }
    }

    std::cout << "A* took " << loopCounter << " iterations" << std::endl;

    // now the required distances are known, but the shortest path is still not (directly) known
    targetNodeAStarElement.position = this->targetPos;
    std::vector<AStarElement> path = {targetNodeAStarElement};

    if (SHOW_WHATS_GOING_ON) this->updateMap(&image);
    while(!path.back().startNode) {
        if (SHOW_WHATS_GOING_ON) image.at<cv::Vec3b>(path.back().position) = cv::Vec3b(127, 127, 127);
        cv::Point2i position = this->dataMap.IndexToXY(path.back().prevIndex);
        AStarElement aStarElement = this->dataMap.getPixel(path.back().prevIndex)->element;
        aStarElement.position = position;
        path.push_back(aStarElement);
    }
    if (SHOW_WHATS_GOING_ON) {
        image.at<cv::Vec3b>(path.back().position) = cv::Vec3b(127, 127, 127);
        cv::imshow("A*", image);
    }

    return path;
}

std::vector <cv::Point2d> FastAStar::aStarListToPointList(std::vector<AStarElement> path) {
    std::vector <cv::Point2d> pointsList;
    for (int i = path.size() - 1; i >= 0; --i) {
        pointsList.push_back(cv::Point2i(
                path[i].position.x * this->mapScaling,
                path[i].position.y * this->mapScaling
        ));
    }
    return pointsList;
}

void FastAStar::run() {
    this->path = this->aStarListToPointList(this->runAStar());
}
