//
// Created by markus on 21.05.21.
//

#ifndef MR_CPP_CODE_MAPFIELD_H
#define MR_CPP_CODE_MAPFIELD_H

#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "constants.h"
#include "Colors.h"


class MapField {
private:
    bool color_left_enabled = false; // TODO: a lot of copy paste -> maybe generalize
    bool color_right_enabled = false;
    bool color_top_enabled = false;
    bool color_bottom_enabled = false;
    cv::Scalar color_left = Colors::white;
    cv::Scalar color_right = Colors::white;
    cv::Scalar color_top = Colors::white;
    cv::Scalar color_bottom = Colors::white;

public:
    cv::Scalar color_bg = Colors::white;
    bool cross = false;
    std::string text;

    void setColorLeft(cv::Scalar color);
    void setColorRight(cv::Scalar color);
    void setColorTop(cv::Scalar color);
    void setColorBottom(cv::Scalar color);

    cv::Mat draw();
};


#endif //MR_CPP_CODE_MAPFIELD_H
