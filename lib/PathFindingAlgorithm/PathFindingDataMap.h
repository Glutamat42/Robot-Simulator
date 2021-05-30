#ifndef MR_CPP_CODE_PATHFINDINGDATAMAP_H
#define MR_CPP_CODE_PATHFINDINGDATAMAP_H

#include "PathFindingDataStructures.h"

template <class T>

class PathFindingDataMap {
private:
    std::vector<PathFindingDatapoint<T>> dataList;
    cv::Point2i bounds;
public:
    PathFindingDataMap() : PathFindingDataMap(0, 0) {}

    PathFindingDataMap(int x, int y) {
        this->bounds = cv::Point2i(x, y);
        this->dataList = std::vector<PathFindingDatapoint<T>>(this->bounds.x * this->bounds.y, PathFindingDatapoint<T>());
    }

    long XYToIndex(int x, int y) {
        return this->bounds.y * y + x;
    }

    cv::Point2i IndexToXY(long index) {
        int y = index / this->bounds.y;
        int x = index % this->bounds.y;
        return cv::Point2i(x,y);
    }

    void setPixelElement(int x, int y, T element) {
        int index = this->bounds.y * y + x;
        if (index >= this->bounds.y * this->bounds.x) throw std::invalid_argument("index is out of map");
        dataList[this->bounds.y * y + x].element = std::move(element);
    }

    void resetData() {
        for (int i = 0; i < this->dataList.size(); ++i) {
            PathFindingDatapoint<T>* datapoint = &this->dataList[i];
            datapoint->element = T();
        }
    }

    PathFindingDatapoint<T>* getPixel(int x, int y) {
        long index = this->bounds.y * y + x;
        return this->getPixel(index);
    }

    PathFindingDatapoint<T>* getPixel(long index) {
        if (index >= this->bounds.y * this->bounds.x) throw std::invalid_argument("index is out of map");
        return &this->dataList[index];
    }

    cv::Point2i getBounds() {
        return this->bounds;
    }
};

#endif //MR_CPP_CODE_PATHFINDINGDATAMAP_H
