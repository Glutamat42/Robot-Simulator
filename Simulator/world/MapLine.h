//
// Created by markus on 22.04.21.
//

#ifndef MR_CPP_CODE_MAPLINE_H
#define MR_CPP_CODE_MAPLINE_H

#include "MapObject.h"

class MapLine : public MapObject {
private:
    std::vector<cv::Point2i> points;
public:
    void draw(cv::Mat image) override;
    void addPoint(cv::Point2i point);
    void clearPoints();
};

#endif //MR_CPP_CODE_MAPLINE_H
