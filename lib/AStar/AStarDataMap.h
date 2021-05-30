////
//// Created by markus on 25.04.21.
////
//
//#ifndef MR_CPP_CODE_ASTARDATAMAP_H
//#define MR_CPP_CODE_ASTARDATAMAP_H
//
//#include "AStarDataStructures.h"
//
//class AStarDataMap {
//private:
//    std::vector<AStarDatapoint> aStarList;
//    cv::Point2i bounds;
//public:
//    AStarDataMap() : AStarDataMap(0, 0) {}
//
//    AStarDataMap(int x, int y) {
//        this->bounds = cv::Point2i(x, y);
//        this->aStarList = std::vector<AStarDatapoint>(this->bounds.x * this->bounds.y, AStarDatapoint());
//    }
//
//    long XYToIndex(int x, int y) {
//        return this->bounds.y * y + x;
//    }
//
//    cv::Point2i IndexToXY(long index) {
//        int y = index / this->bounds.y;
//        int x = index % this->bounds.y;
//        return cv::Point2i(x,y);
//    }
//
//    void setPixelAstarElement(int x, int y, AStarElement element) {
//        int index = this->bounds.y * y + x;
//        if (index >= this->bounds.y * this->bounds.x) throw std::invalid_argument("index is out of map");
//        aStarList[this->bounds.y * y + x].aStarElement = std::move(element);
//    }
//
//    void resetAStarData() {
//        for (int i = 0; i < this->aStarList.size(); ++i) {
//            AStarDatapoint* datapoint = &this->aStarList[i];
//            datapoint->aStarElement = AStarElement();
//        }
//    }
//
//    AStarDatapoint* getPixel(int x, int y) {
//        long index = this->bounds.y * y + x;
//        return this->getPixel(index);
//    }
//
//    AStarDatapoint* getPixel(long index) {
//        if (index >= this->bounds.y * this->bounds.x) throw std::invalid_argument("index is out of map");
//        return &aStarList[index];
//    }
//
//    cv::Point2i getBounds() {
//        return this->bounds;
//    }
//};
//
//#endif //MR_CPP_CODE_ASTARDATAMAP_H
