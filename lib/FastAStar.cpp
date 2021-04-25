//
// Created by markus on 23.04.21.
//

#include "FastAStar.h"
#include "helpers.h"

FastAStar::FastAStar(std::string map_filename, double paddingRadius) {
    // load map, generate fastmap and padded map, convert padded map to mat again and show it
    cv::Mat grayscaleMap = cv::imread(map_filename, cv::IMREAD_GRAYSCALE);
    if ((grayscaleMap.cols == 0) || (grayscaleMap.rows == 0)) {
        throw std::invalid_argument("Error! Could not read the image file: '" + map_filename + "'");
    }

    FastMap paddedMap = padObstacles(FastMap(grayscaleMap), paddingRadius);
    this->scaledAndPaddedMap = paddedMap.getDownScaledMap(MAP_SCALING);
    // This approach is a little faster but the result works not so good
//    this->scaledAndPaddedMap = padObstacles(FastMap(grayscaleMap).getDownScaledMap(MAP_SCALING),paddingRadius / MAP_SCALING);

    if (SHOW_WHATS_GOING_ON) {
        cv::Mat paddedImage = this->scaledAndPaddedMap.toCVMat();
        cv::imshow("padded image", paddedImage);
        cv::waitKey(1);
    }

    this->scaledMapBounds = this->scaledAndPaddedMap.getBounds();
    this->aStarMap = AStarMap(this->scaledMapBounds.x, this->scaledMapBounds.y);
    this->generateAdjacencyList();
}

const int neighborsDiagonal[4][2] = {{-1, -1},
                                     {-1, 1},
                                     {1,  -1},
                                     {1,  1}};
const int neighborsDirect[4][2] = {{0,  -1},
                                   {-1, 0},
                                   {1,  0},
                                   {0,  1}};

void FastAStar::generateAdjacencyList() {
    // generate list. Skipping the outermost pixels to avoid a boundary-check-if. Those pixels cant be used anyways because our robot will have a size greater than 0.5
    for (int x = 1; x < this->scaledMapBounds.x - 1; ++x) {
        for (int y = 1; y < this->scaledMapBounds.y - 1; ++y) {
            // skip if current pixel is a wall
            if (this->scaledAndPaddedMap.getPixel(x, y)) continue;

            AStarDatapoint* curDataPoint = this->aStarMap.getPixel(x,y);
            // if neighbors are no boarder add them to the adjacency list
            for (auto &i : neighborsDiagonal) {
                if (!this->scaledAndPaddedMap.getPixel(x + i[0], y + i[1]))
                    curDataPoint->adjacencyTargets.push_back(AdjacencyTarget({this->aStarMap.XYToIndex(x + i[0], y + i[1]), 1.41421}));
            }
            for (auto &i : neighborsDirect) {
                if (!this->scaledAndPaddedMap.getPixel(x + i[0], y + i[1]))
                    curDataPoint->adjacencyTargets.push_back(AdjacencyTarget({this->aStarMap.XYToIndex(x + i[0], y + i[1]), 1.0}));
            }
        }
    }
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
    cv::Point2i _startPos = startPosition / MAP_SCALING;
    cv::Point2i _targetPos = targetPosition / MAP_SCALING;

    if (this->scaledAndPaddedMap.getPixel(_startPos.x, _startPos.y) || this->scaledAndPaddedMap.getPixel(_targetPos.x, _targetPos.y)) {
        std::cout << "invalid start or target position" << std::endl;
        return false;
    }

    this->startPos = _startPos;
    this->targetPos = _targetPos;
    this->targetIndex = this->aStarMap.XYToIndex(this->targetPos.x, this->targetPos.y);
    this->heuristicBias = bias;

    if (bias != 1.0) std::cout << "Using heuristic bias of " << this->heuristicBias << std::endl;

    this->aStarMap.resetAStarData();
//    this->aStarListOpen.clear();
    this->aStarMapOpen.clear();

    // add startPos to openList
    this->aStarMap.setPixelAstarElement(
            this->startPos.x,
            this->startPos.y,
            AStarElement({this->aStarMap.XYToIndex(this->startPos.x, this->startPos.y), 0, 0, 0, true, true, false, cv::Point2i()}));
//    this->aStarListOpen.push_back(this->aStarMap.XYToIndex(this->startPos.x, this->startPos.y));
    this->aStarMapOpen.insert(std::pair<std::tuple<double, double, long>, long>(std::tuple(0,0,0), this->aStarMap.XYToIndex(this->startPos.x, this->startPos.y)));

    return true;
}

std::vector <AStarElement> FastAStar::runAStar() {
    cv::Mat image;
    int weightColorDivider = this->scaledMapBounds.x * 0.6;  // simply looks not bad on my example

    int loopCounter = 0;
    AStarElement targetNodeAStarElement;

    if (SHOW_WHATS_GOING_ON) {
        cv::cvtColor(this->scaledAndPaddedMap.toCVMat(), image, cv::COLOR_GRAY2RGB);
    }

    while (true) {
        if (SHOW_WHATS_GOING_ON && loopCounter % 5000 == 0) {
            // show current open and closed list on map
            for (int x = 0; x <this->scaledMapBounds.x; ++x) {
                for (int y = 0; y <this->scaledMapBounds.y; ++y) {
                    AStarDatapoint* aStarDatapoint = this->aStarMap.getPixel(x,y);
                    if (aStarDatapoint->aStarElement.isOpenList)
                        image.at<cv::Vec3b>(cv::Point2i(x,y)) = cv::Vec3b(0, 255, 0);
                    if (aStarDatapoint->aStarElement.isClosedList)
                        image.at<cv::Vec3b>(cv::Point2i(x,y)) = cv::Vec3b(0, 0, 255 * (int) aStarDatapoint->aStarElement.f_cost / weightColorDivider);
                }
            }
            image.at<cv::Vec3b>(this->startPos) = cv::Vec3b(255, 0, 0);
            image.at<cv::Vec3b>(this->targetPos) = cv::Vec3b(255, 255, 255);
            cv::imshow("A*", image);
            cv::waitKey(1);
        }
        loopCounter++;


//        // find element with lowest f_cost, remove it from openList and add to closedList
//        long lowestFcostIndex = this->aStarListOpen[0];
//        int lowestFcostIndexListIndex = 0;
//        AStarDatapoint* currentNode = this->aStarMap.getPixel(lowestFcostIndex);
//        for (int i = 0; i < this->aStarListOpen.size(); ++i) {
//            if (this->aStarMap.getPixel(this->aStarListOpen[i])->aStarElement.f_cost < currentNode->aStarElement.f_cost) {
//                currentNode = this->aStarMap.getPixel(this->aStarListOpen[i]);
//                lowestFcostIndex = this->aStarListOpen[i];
//                lowestFcostIndexListIndex = i;
//            } // TODO: if openList[i].f_cost == currentNode.f_cost -> lower heuristic?
//        }
//        this->aStarListOpen.erase(this->aStarListOpen.begin() + lowestFcostIndexListIndex);

        long lowestFcostIndex = this->aStarMapOpen.begin()->second;
        this->aStarMapOpen.erase(this->aStarMapOpen.begin());
        AStarDatapoint* currentNode = this->aStarMap.getPixel(lowestFcostIndex);

        currentNode->aStarElement.isOpenList = false;
        currentNode->aStarElement.isClosedList = true;


        // check if the currentNode node is the target
        if (lowestFcostIndex == this->targetIndex) {
            targetNodeAStarElement = currentNode->aStarElement;
            break;
        };


        // iterate through neighbors
        for (AdjacencyTarget &curNeighborAdjacencyEntry : currentNode->adjacencyTargets) {
            // check if neighbor is closed -> if yes skip this node (nothing to do for this node)
            if (this->aStarMap.getPixel(curNeighborAdjacencyEntry.index)->aStarElement.isClosedList) continue;

            // calculate distance from start over current node to current neighbor
            double newDistance = currentNode->aStarElement.distance + curNeighborAdjacencyEntry.weight;

            // search for currentNeighbor in openList
            AStarDatapoint* currentNeighborDatapoint = this->aStarMap.getPixel(curNeighborAdjacencyEntry.index);
            bool isInOpenList = currentNeighborDatapoint->aStarElement.isOpenList;

            // if entry found in open list, check if newDistance is greater than the previous found path to neighbor
            if (isInOpenList && newDistance >= currentNeighborDatapoint->aStarElement.distance) {
                // old way is better or equal -> skip
                continue;
            }

            // if not yet in openlist: add entry and calculate heuristic (h_cost)
            if (!isInOpenList) {
//                this->aStarListOpen.push_back(curNeighborAdjacencyEntry.index);

                cv::Point2i heuristic_distance = this->aStarMap.IndexToXY(curNeighborAdjacencyEntry.index) - this->targetPos;
                double heuristic = sqrt(heuristic_distance.x * heuristic_distance.x + heuristic_distance.y * heuristic_distance.y);
                currentNeighborDatapoint->aStarElement.heuristic = heuristic;
//                currentNeighborDatapoint->aStarElement.prevIndex = lowestFcostIndex;
            }

            double old_f_cost = currentNeighborDatapoint->aStarElement.f_cost;
            // calculate and set distance (g_cost) and f_cost
            currentNeighborDatapoint->aStarElement.distance = newDistance;
            currentNeighborDatapoint->aStarElement.f_cost = currentNeighborDatapoint->aStarElement.distance + currentNeighborDatapoint->aStarElement.heuristic * this->heuristicBias;
            currentNeighborDatapoint->aStarElement.prevIndex = lowestFcostIndex;

            if (!isInOpenList) {
                currentNeighborDatapoint->aStarElement.isOpenList = true;
                while(!this->aStarMapOpen.insert(std::pair<std::tuple<double,double, long>, long>(std::tuple<double,double, long> (currentNeighborDatapoint->aStarElement.f_cost, currentNeighborDatapoint->aStarElement.heuristic, curNeighborAdjacencyEntry.index), curNeighborAdjacencyEntry.index)).second) {
                    currentNeighborDatapoint->aStarElement.heuristic = std::nextafter(currentNeighborDatapoint->aStarElement.heuristic, std::numeric_limits<double>::max());
                }

            } else {

//                if (curNeighborAdjacencyEntry.index == 62189) {
//                    for (auto it = this->aStarMapOpen.begin(); it != this->aStarMapOpen.end(); ++it)
//                        if (it->second == 62189) {
//                            auto test2 = it->second;
//                            auto test1 = it->first;
//                            auto test = it;
//                            std::cout << test1;
//                        }
//                }

                auto nodeHandler = this->aStarMapOpen.extract(std::tuple<double, double, long>(old_f_cost, currentNeighborDatapoint->aStarElement.heuristic, curNeighborAdjacencyEntry.index));
                nodeHandler.key() = std::tuple<double, double, long>(currentNeighborDatapoint->aStarElement.f_cost, currentNeighborDatapoint->aStarElement.heuristic, curNeighborAdjacencyEntry.index);
                this->aStarMapOpen.insert(std::move(nodeHandler));
            }
        }
    }

    std::cout << "A* took " << loopCounter << " iterations" << std::endl;

    // now the required distances are known, but the shortest path is still not (directly) known
    targetNodeAStarElement.position = this->targetPos;
    std::vector<AStarElement> path = {targetNodeAStarElement};

    while(!path.back().startNode) {
        if (SHOW_WHATS_GOING_ON) image.at<cv::Vec3b>(path.back().position) = cv::Vec3b(127, 127, 127);
        cv::Point2i position = this->aStarMap.IndexToXY(path.back().prevIndex);
        AStarElement aStarElement = this->aStarMap.getPixel(path.back().prevIndex)->aStarElement;
        aStarElement.position = position;
        path.push_back(aStarElement);
    }
    if (SHOW_WHATS_GOING_ON) {
        image.at<cv::Vec3b>(path.back().position) = cv::Vec3b(127, 127, 127);
        cv::imshow("A*", image);
    }

    return path;
}

std::vector <cv::Point2d> FastAStar::aStarListToPointList(std::vector <AStarElement> path) {
    std::vector <cv::Point2d> pointsList;
    for (int i = path.size() - 1; i >= 0; --i) {
        pointsList.push_back(cv::Point2i(
                path[i].position.x * MAP_SCALING,
                path[i].position.y * MAP_SCALING
        ));
    }
    return pointsList;
}
