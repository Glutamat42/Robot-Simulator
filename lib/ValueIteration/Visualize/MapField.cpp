//
// Created by markus on 21.05.21.
//

#include <iostream>
#include "MapField.h"

cv::Mat MapField::draw() {
    int imageWidth = (FIELD_WIDTH + 2 * BORDER_WIDTH + 1);
    int imageHeight = (FIELD_HEIGHT + 2 * BORDER_WIDTH + 1);
    cv::Mat img = cv::Mat::zeros(imageHeight, imageWidth, CV_8UC3);

    // bg
    cv::rectangle(img,cv::Point2i (0,0),cv::Point2i (imageWidth,imageHeight),this->color_bg,cv::FILLED);

    // borders
    if(this->color_left_enabled) {
        cv::rectangle(img, cv::Point2i(0, 0), cv::Point2i(BORDER_WIDTH, imageHeight), this->color_left, cv::FILLED);
    }
    if(this->color_right_enabled) {
        cv::rectangle(img, cv::Point2i(BORDER_WIDTH + FIELD_WIDTH, 0), cv::Point2i(imageWidth, imageHeight), this->color_right, cv::FILLED);
    }
    if(this->color_top_enabled) {
        cv::rectangle(img, cv::Point2i(0, 0), cv::Point2i(imageWidth, BORDER_WIDTH), this->color_top, cv::FILLED);
    }
    if(this->color_bottom_enabled) {
        cv::rectangle(img, cv::Point2i(0, BORDER_WIDTH + FIELD_HEIGHT), cv::Point2i(imageWidth, imageHeight), this->color_bottom, cv::FILLED);
    }

    // text
    if (!this->text.empty()) {
        cv::putText(img, this->text, cv::Point2i(BORDER_WIDTH + 1,  FIELD_HEIGHT), cv::FONT_HERSHEY_SIMPLEX, 0.4, Colors::black, 1);

    }

    // cross
    if (this->cross) {
        cv::line(img, cv::Point2i(BORDER_WIDTH + 3, BORDER_WIDTH + 3), cv::Point2i(BORDER_WIDTH + FIELD_WIDTH - 3, BORDER_WIDTH + FIELD_HEIGHT - 3),
                 cv::Scalar(0, 0, 0), 2);
        cv::line(img, cv::Point2i(BORDER_WIDTH + FIELD_WIDTH - 3, BORDER_WIDTH + 3), cv::Point2i(BORDER_WIDTH + 3, BORDER_WIDTH + FIELD_HEIGHT - 3),
                 cv::Scalar(0, 0, 0), 2);
    }

    return img;
}

void MapField::setColorLeft(cv::Scalar color) {
    this->color_left_enabled = true;
    this->color_left = color;
}

void MapField::setColorRight(cv::Scalar color) {
    this->color_right_enabled = true;
    this->color_right = color;
}

void MapField::setColorTop(cv::Scalar color) {
    this->color_top_enabled = true;
    this->color_top = color;
}

void MapField::setColorBottom(cv::Scalar color) {
    this->color_bottom_enabled = true;
    this->color_bottom = color;
}
