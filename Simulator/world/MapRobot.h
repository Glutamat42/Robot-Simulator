//
// Created by markus on 24.04.21.
//

#ifndef MR_CPP_CODE_MAPROBOT_H
#define MR_CPP_CODE_MAPROBOT_H


#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include "MapObject.h"

class MapRobot : public MapObject  {
private:
    double radius;
    double angle;
    cv::Point2d pos;
    cv::Scalar robotCircleColor = CV_RGB(255, 0, 0);
public:
    explicit MapRobot(double radius);
    MapRobot(double radius, double angle, cv::Point2d pos);
    void reposition(cv::Point2d pos, double angle);
    void draw(cv::Mat image) override;
    void setColor(cv::Scalar color);
};


#endif //MR_CPP_CODE_MAPROBOT_H
