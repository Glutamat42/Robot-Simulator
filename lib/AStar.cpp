//
// Created by markus on 23.04.21.
//

#include "AStar.h"
#include "helpers.h"

AStar::AStar(std::string map_filename, double paddingRadius) {
    // load map, generate fastmap and padded map, convert padded map to mat again and show it
    cv::Mat grayscaleMap = cv::imread(map_filename, cv::IMREAD_GRAYSCALE);
    if ((grayscaleMap.cols == 0) || (grayscaleMap.rows == 0)) {
        throw std::invalid_argument("Error! Could not read the image file: '" + map_filename + "'");
    }

//    this->map = FastMap(grayscaleMap);

    this->scaledAndPaddedMap = padObstacles(
            FastMap(grayscaleMap).getDownScaledMap(MAP_SCALING),
            8.0 / MAP_SCALING);

    if (SHOW_WHATS_GOING_ON) {
        cv::Mat paddedImage = this->scaledAndPaddedMap.toCVMat();
        cv::imshow("padded image", paddedImage);
        cv::waitKey(1);
    }

    this->adjacencyList = this->generateAdjacencyList(this->scaledAndPaddedMap);
}

const int neighborsDiagonal[4][2] = {{-1, -1},
                                     {-1, 1},
                                     {1,  -1},
                                     {1,  1}};
const int neighborsDirect[4][2] = {{0,  -1},
                                   {-1, 0},
                                   {1,  0},
                                   {0,  1}};

AdjacencyList AStar::generateAdjacencyList(FastMap map) {
    AdjacencyList list;

    int numDigitsForXMulti = map.getBounds().x == 0 ? 1 : log10(std::abs(map.getBounds().x)) + 1;
    list.searchIndexXMultiplier = pow(10, numDigitsForXMulti);

    // generate list. Skipping the outermost pixels to avoid a boundary-check-if. Those pixels cant be used anyways because our robot will have a size greater than 0.5
    for (int x = 1; x < map.getBounds().x - 1; ++x) {
        for (int y = 1; y < map.getBounds().y - 1; ++y) {
            // skip if current pixel is a wall
            if (map.getPixel(x, y)) continue;

            // if neighbors are no boarder add them to the adjacency list
            AdjacencyRow newListEntry({cv::Point2i(x, y), {}});
            for (auto &i : neighborsDiagonal) {
                if (!map.getPixel(x + i[0], y + i[1]))
                    newListEntry.to.push_back((AdjacencyTarget) {cv::Point2i(x + i[0], y + i[1]), 1.41421});
            }
            for (auto &i : neighborsDirect) {
                if (!map.getPixel(x + i[0], y + i[1]))
                    newListEntry.to.push_back((AdjacencyTarget) {cv::Point2i(x + i[0], y + i[1]), 1.0});
            }
            list.list.push_back(newListEntry);
            list.searchIndex.push_back(x * list.searchIndexXMultiplier + y);
        }
    }
    std::cout << "adjacency list has " << list.list.size() << " entries" << std::endl;

    // bubble sort, could be replaced with a more efficient algorithm if its computationally too expensive
    bool finished = false;
    for (int i = 0; !finished && i < list.list.size(); ++i) {
        finished = true;
        for (int j = i + 1; j < list.list.size(); ++j) {
            if (list.searchIndex[j] < list.searchIndex[i]) {
                finished = false;
                AdjacencyRow tmpAdjacencyRow = list.list[j];
                list.list[j] = list.list[i];
                list.list[i] = tmpAdjacencyRow;
                int tmpAdjacencyListSearchIndex = list.searchIndex[j];
                list.searchIndex[j] = list.searchIndex[i];
                list.searchIndex[i] = tmpAdjacencyListSearchIndex;
            }
        }
    }

    return list;
}


const int neighbors[8][2] = {{-1, -1},
                             {-1, 1},
                             {1,  -1},
                             {1,  1},
                             {0,  -1},
                             {-1, 0},
                             {1,  0},
                             {0,  1}};

void AStar::setAStarParameters(cv::Point2i startPosition, cv::Point2i targetPosition, double bias) {
    this->startPos = cv::Point2i(startPosition.x / MAP_SCALING, startPosition.y / MAP_SCALING);
    this->targetPos = cv::Point2i(targetPosition.x / MAP_SCALING, targetPosition.y / MAP_SCALING);
    this->heuristicBias = bias;

    if (bias != 1.0) std::cout << "Using heuristic bias of " << this->heuristicBias << std::endl;

    this->openList.clear();
    this->closedList.clear();

    // add startPos to openList
    this->openList.push_back((AStarElement) {this->startPos, cv::Point2i(), 0, 0, 0, true});
}

std::vector <AStarElement> AStar::runAStar() {
    cv::Mat image;
    int weightColorDivider = this->scaledAndPaddedMap.getBounds().x * 0.6;  // simply looks not bad on my example

    int loopCounter = 0;
    AStarElement targetNodeAStarElement;

    if (SHOW_WHATS_GOING_ON) {
        cv::cvtColor(this->scaledAndPaddedMap.toCVMat(), image, cv::COLOR_GRAY2RGB);
    }

    while (true) {
        if (SHOW_WHATS_GOING_ON && loopCounter % 100 == 0) {
            // show current open and closed list on map
            // inefficient, but "good enough"
            for (const auto &entry : closedList) {
                image.at<cv::Vec3b>(entry.position) = cv::Vec3b(0, 0, 255 * (int) entry.f_cost / weightColorDivider);
            }
            for (const auto &entry : openList) {
                image.at<cv::Vec3b>(entry.position) = cv::Vec3b(0, 255, 0);
            }
            image.at<cv::Vec3b>(this->startPos) = cv::Vec3b(255, 0, 0);
            image.at<cv::Vec3b>(this->targetPos) = cv::Vec3b(255, 255, 255);
            cv::imshow("A*", image);
            cv::waitKey(1);
        }
        loopCounter++;


        // find element with lowest f_cost, remove it from openList and add to closedList
        AStarElement currentNode = openList[0];
        int index = 0;
        for (int i = 0; i < openList.size(); ++i) {
            if (openList[i].f_cost < currentNode.f_cost) {
                currentNode = openList[i];
                index = i;
            } // TODO: if openList[i].f_cost == currentNode.f_cost -> lower heuristic?
        }
        openList.erase(openList.begin() + index);
        closedList.push_back(currentNode);

        // check if the currentNode node is the target
        if (currentNode.position == this->targetPos) {
            targetNodeAStarElement = currentNode;
            break;
        };

        // find adjacency entry for currentNode node
        AdjacencyRow currentRow;
        auto lower = std::lower_bound(this->adjacencyList.searchIndex.begin(),
                                      this->adjacencyList.searchIndex.end(),
                                      currentNode.position.x * this->adjacencyList.searchIndexXMultiplier + currentNode.position.y);
        long indexOfCurrentRow = std::distance(this->adjacencyList.searchIndex.begin(), lower);
        if (indexOfCurrentRow == this->adjacencyList.searchIndex.size()) continue; // no entry for current element in adjacency list
        currentRow = this->adjacencyList.list[indexOfCurrentRow];

        // iterate through neighbors
        for (AdjacencyTarget &curNeighborAdjacencyEntry : currentRow.to) {
            // check if neighbor is closed -> if yes skip this node (nothing to do for this node)
            bool isInClosed = false;
            for (AStarElement node : closedList) { // the bigger the closed list gets the more inefficient this will become -> this search algorithm will not work for larger lists!
                if (node.position == curNeighborAdjacencyEntry.position) {
                    isInClosed = true;
                    break;
                }
            }
            if (isInClosed) continue;

            // calculate distance from start over current node to current neighbor
            double newDistance = currentNode.distance + curNeighborAdjacencyEntry.weight;

            // search for currentNeighbor in openList
            long openListIndex = -1;
            for (int i = 0; i < openList.size(); ++i) {
                if (openList[i].position == curNeighborAdjacencyEntry.position) {
                    openListIndex = i;
                    break;
                }
            }
            // if entry found in open list, check if newDistance is greater than the previous found path to neighbor
            if (openListIndex >= 0 && newDistance >= openList[openListIndex].distance) {
                // old way is better or equal -> skip
                continue;
            }

            // if not yet in openlist: add entry and calculate heuristic (h_cost)
            if (openListIndex == -1) {
                openListIndex = (long) openList.size();
                cv::Point2i heuristic_distance = curNeighborAdjacencyEntry.position - this->targetPos;
                double heuristic = sqrt(heuristic_distance.x * heuristic_distance.x + heuristic_distance.y * heuristic_distance.y);
                openList.push_back((AStarElement) {curNeighborAdjacencyEntry.position, currentNode.position, heuristic, 0, 0});
            }

            // calculate and set distance (g_cost) and f_cost
            openList[openListIndex].distance = newDistance;
            openList[openListIndex].f_cost = openList[openListIndex].distance + openList[openListIndex].heuristic * this->heuristicBias;
            openList[openListIndex].prev = currentNode.position;
        }
    }

    std::cout << "A* took " << loopCounter << " iterations" << std::endl;

    // now the required distances are known, but the shortest path is still not (directly) known
    std::vector<AStarElement> path = {targetNodeAStarElement};
    while (!path.back().startNode) {
        if (SHOW_WHATS_GOING_ON) image.at<cv::Vec3b>(path.back().position) = cv::Vec3b(127, 127, 127);
        for (const AStarElement &element : closedList) {
            if (element.position == path.back().prev) {
                path.push_back(element);
                break;
            }
        }
    }
    if (SHOW_WHATS_GOING_ON) {
        image.at<cv::Vec3b>(path.back().position) = cv::Vec3b(127, 127, 127);
        cv::imshow("A*", image);
    }

    return path;
}

std::vector <cv::Point2i> AStar::aStarListToPointList(std::vector <AStarElement> path) {
    std::vector <cv::Point2i> pointsList;
    for (int i = path.size() - 1; i >= 0; --i) {
        pointsList.push_back(cv::Point2i(
                path[i].position.x * MAP_SCALING,
                path[i].position.y * MAP_SCALING
        ));
    }
    return pointsList;
}
