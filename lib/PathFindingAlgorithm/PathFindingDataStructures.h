//
// Created by markus on 29.05.21.
//

#ifndef MR_CPP_CODE_PATHFINDINGDATASTRUCTURES_H
#define MR_CPP_CODE_PATHFINDINGDATASTRUCTURES_H

struct AdjacencyTarget {
    long index;
    double weight;
};

template <class T>
struct PathFindingDatapoint {
    std::vector<AdjacencyTarget> adjacencyTargets;
//    std::vector<AdjacencyTarget> adjacencyTargets = std::vector<AdjacencyTarget>(8);  // creating adjacency List can be sped up quite a bit by reserving memory first, but this will also significantly increase the memory consumption

    T element = T();
};

#endif //MR_CPP_CODE_PATHFINDINGDATASTRUCTURES_H
