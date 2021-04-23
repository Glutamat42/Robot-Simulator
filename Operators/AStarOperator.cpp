//
// Created by markus on 22.04.21.
//

#include "AStarOperator.h"
#include "../lib/helpers.h"

AStarOperator::AStarOperator(RobotControlInterface *robot, std::string map_filename) : RobotOperator(robot) {
    // load map, generate fastmap and padded map, convert padded map to mat again and show it
    cv::Mat grayscaleMap = cv::imread(map_filename, cv::IMREAD_GRAYSCALE);
    if ((grayscaleMap.cols == 0) || (grayscaleMap.rows == 0)) {
        throw std::invalid_argument("Error! Could not read the image file: '" + map_filename + "'");
    }

    FastMap map = FastMap(grayscaleMap);

    int downScaling = 6;
    FastMap paddedMap = padObstacles(map.getDownScaledMap(downScaling), 8.0 / downScaling);

    cv::Mat paddedImage = paddedMap.toCVMat();
    cv::imshow("padded image", paddedImage);
    cv::waitKey(1);
    // END load map, generate fastmap and padded map, convert padded map to mat again and show it

    // Adjacency list
    struct adjacencyTarget {
        cv::Point2i position;
        double weight;
    };
    struct adjacencyRow {
        cv::Point2i from;
        std::vector<adjacencyTarget> to;
    };
    int numDigitsForXMulti = paddedMap.getBounds().x == 0 ? 1 : log10(std::abs(paddedMap.getBounds().x)) + 1;
    int indexXMultiplier = pow(10, numDigitsForXMulti);

    std::vector<adjacencyRow> adjacencyList;
    std::vector<int> adjacencyListSearchIndex;

    int neighborsDiagonal[4][2] = {{-1, -1},
                                   {-1, 1},
                                   {1,  -1},
                                   {1,  1}};
    int neighborsDirect[4][2] = {{0, -1},
                                 {-1, 0},
                                 {1, 0},
                                 {0, 1}};
    // generate list. Skipping the outermost pixels to avoid a boundary-check-if. Those pixels cant be used anyways because our robot will have a size greater than 0.5
    for (int x = 1; x < paddedMap.getBounds().x - 1; ++x) {
        for (int y = 1; y < paddedMap.getBounds().y - 1; ++y) {
            // skip if current pixel is a wall
            if (paddedMap.getPixel(x, y)) continue;

            // if neighbors are no boarder add them to the adjacency list
            adjacencyRow newListEntry({cv::Point2i(x, y), {}});
            for (auto &i : neighborsDiagonal) {
                if (!paddedMap.getPixel(x + i[0], y + i[1]))
                    newListEntry.to.push_back((adjacencyTarget) {cv::Point2i(x + i[0], y + i[1]), 1.41421});
            }
            for (auto &i : neighborsDirect) {
                if (!paddedMap.getPixel(x + i[0], y + i[1]))
                    newListEntry.to.push_back((adjacencyTarget) {cv::Point2i(x + i[0], y + i[1]), 1.0});
            }
            adjacencyList.push_back(newListEntry);
            if (newListEntry.from.x == 0)
                std::cout << "shit";
            adjacencyListSearchIndex.push_back(x * indexXMultiplier + y);
        }
    }
    std::cout << "adjacency list has " << adjacencyList.size() << " entries" << std::endl;

    // sort list
    // bubble sort
    // could be replaced with a more efficient algorithm if its computationally too expensive
    bool finished = false;
    for (int i = 0; !finished && i < adjacencyList.size(); ++i) {
        finished = true;
        for (int j = i + 1; j < adjacencyList.size(); ++j) {
            if (adjacencyListSearchIndex[j] < adjacencyListSearchIndex[i]) {
                finished = false;
                adjacencyRow tmpAdjacencyRow = adjacencyList[j];
                adjacencyList[j] = adjacencyList[i];
                adjacencyList[i] = tmpAdjacencyRow;
                int tmpAdjacencyListSearchIndex = adjacencyListSearchIndex[j];
                adjacencyListSearchIndex[j] = adjacencyListSearchIndex[i];
                adjacencyListSearchIndex[i] = tmpAdjacencyListSearchIndex;
            }
        }
    }
    // END Adjacency list

    // --------------------------
    // A*
    // set coordinates
    cv::Point2i startLocation(310 / downScaling, 300 / downScaling); // TODO get this from robot/particle filter
//    cv::Point2i targetLocation(190 / downScaling, 230 / downScaling);
    cv::Point2i targetLocation(750 / downScaling, 700 / downScaling); // longer and more difficult target

    // prepare datastructures and lists
    struct AStarElement {
        cv::Point2i position;
        cv::Point2i prev;
        double heuristic;
        double distance;
        double f_cost;
        bool startNode = false;
    };
    std::vector<AStarElement> openList;
    std::vector<AStarElement> closedList;
    int neighbors[8][2] = {{-1, -1},
                           {-1, 1},
                           {1,  -1},
                           {1,  1},
                           {0,  -1},
                           {-1, 0},
                           {1,  0},
                           {0,  1}};

    // begin algorithm
    openList.push_back((AStarElement) {startLocation, cv::Point2i(), 0, 0, 0, true});

    AStarElement targetNodeAStarElement;

    // ui stuff
    cv::Mat image;
    cv::cvtColor(paddedImage, image, cv::COLOR_GRAY2RGB);
    int loopCounter = 0;
    // END ui stuff
#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
    while (true) {
        if (loopCounter % 50 == 0) {
            // show current open and closed list on map
            // horrible inefficient, just for testing purposes for now
            for (const auto &entry : closedList) {
                image.at<cv::Vec3b>(entry.position) = cv::Vec3b(0, 0, 255);
            }
            for (const auto &entry : openList) {
                image.at<cv::Vec3b>(entry.position) = cv::Vec3b(0, 255, 0);
            }
            image.at<cv::Vec3b>(startLocation) = cv::Vec3b(255, 0, 0);
            image.at<cv::Vec3b>(targetLocation) = cv::Vec3b(255, 255, 255);
            std::cout << "loop counter: " << loopCounter << std::endl;
            cv::imshow("A*", image);
            cv::waitKey(1);
        }
        loopCounter++;


        // find lowest f_cost
        AStarElement currentNode = openList[0];
        int index = 0;
        for (int i = 0; i < openList.size(); ++i) {
            if (openList[i].f_cost < currentNode.f_cost) {
                currentNode = openList[i];
                index = i;
            } // TODO: if openList[i].f_cost == currentNode.f_cost -> lower heuristic?
        }

        // remove currentNode from openlist
        openList.erase(openList.begin() + index);

        // add currentNode element to closedList
        closedList.push_back(currentNode);

        // check if the currentNode node is the target
        if (currentNode.position == targetLocation) {
            targetNodeAStarElement = currentNode;
            break;
        };

        // find adjacency entry for currentNode node
        adjacencyRow currentRow;
        auto lower = std::lower_bound(adjacencyListSearchIndex.begin(), adjacencyListSearchIndex.end(),
                                      currentNode.position.x * indexXMultiplier + currentNode.position.y);
        int indexOfCurrentRow = std::distance(adjacencyListSearchIndex.begin(), lower);
        if (indexOfCurrentRow == adjacencyListSearchIndex.size()) std::cout << "NOT FOUND" << std::endl;
        currentRow = adjacencyList[indexOfCurrentRow];
//        for (AdjacencyRow row : list) {
//            if (row.from == currentNode.position) {
//                currentRow = row;
//                break;
//            }
//        }
        if (currentRow.from == cv::Point2i()) {
            continue; // no entry for current element in adjacency list
        }

        // iterate through neighbors
        for (adjacencyTarget &curNeighborAdjacencyEntry : currentRow.to) {
            // check if neighbor is closed
            bool isInClosed = false;
            for (AStarElement node : closedList) {
                if (node.position == curNeighborAdjacencyEntry.position) {
                    isInClosed = true;
                    break;
                }
            }
            if (isInClosed) continue;

            // calculate distance from start over current node to current neighbor
            double newDistance = currentNode.distance + curNeighborAdjacencyEntry.weight;

            // search for currentNeighbor in openList
            int openListIndex = -1;
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
                openListIndex = openList.size();
                cv::Point2i heuristic_distance = curNeighborAdjacencyEntry.position - targetLocation;
                double heuristic = sqrt(heuristic_distance.x * heuristic_distance.x + heuristic_distance.y * heuristic_distance.y);
                openList.push_back((AStarElement) {curNeighborAdjacencyEntry.position, currentNode.position, heuristic, 0, 0});
            }

            // calculate and set distance (g_cost) and f_cost
            openList[openListIndex].distance = newDistance;
            openList[openListIndex].f_cost = openList[openListIndex].distance + openList[openListIndex].heuristic;
            openList[openListIndex].prev = currentNode.position;
        }
    }

    AStarElement currentPoint = targetNodeAStarElement;
    while (!currentPoint.startNode) {
        image.at<cv::Vec3b>(currentPoint.position) = cv::Vec3b(127, 127, 127);
        for (const AStarElement& element : closedList) {
            if (element.position == currentPoint.prev) {
                currentPoint = element;
                break;
            }
        }
    }
    cv::imshow("A*", image);
#pragma clang diagnostic pop
//    exit(0);
    cv::waitKey(0);
    std::cout << "HELLO WORLD" << std::endl;
}

void AStarOperator::update() {

}
