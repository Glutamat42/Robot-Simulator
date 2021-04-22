//
// Created by markus on 22.04.21.
//

#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include "MapLine.h"

void MapLine::clearPoints() {
    this->points.clear();
}

void MapLine::addPoint(cv::Point2i point) {
    this->points.push_back(point);
}

void MapLine::draw(cv::Mat image) {
    if (this->points.size() <= 1) {
        return;
    }
    for (int i = 1; i<this->points.size(); ++i) {
        cv::line(image, this->points[i-1], this->points[i], cv::Scalar(123,123,123), 2);
    }
}