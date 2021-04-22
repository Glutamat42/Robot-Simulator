//
// Created by markus on 22.04.21.
//

#ifndef MR_CPP_CODE_FASTMAP_H
#define MR_CPP_CODE_FASTMAP_H

/** a simple structure for a 2d map object with only boolean values */
class FastMap {
private:
    std::vector<bool> fastMap;
    cv::Point2i mapBounds;
public:
    FastMap(int x, int y, bool initialValue = false) {
        this->mapBounds = cv::Point2i(x,y);
        this->fastMap = std::vector<bool>(this->mapBounds.x * this->mapBounds.y, initialValue);
    }

    FastMap(cv::Mat grayscaleImage) {
        this->mapBounds = cv::Point2i(grayscaleImage.cols, grayscaleImage.rows);
        this->fastMap = std::vector<bool>(this->mapBounds.x * this->mapBounds.y);
        for (int y = 0; y < this->mapBounds.y; y++) {
            for (int x = 0; x < this->mapBounds.x; x++) {
                this->setFastMapPixel(x, y, grayscaleImage.at<uchar>(y, x) != 0);
            }
        }
    }

    void setFastMapPixel(int x, int y, bool value) {
        fastMap[mapBounds.y * y + x] = value;
    }
    bool getFastMapPixel(int x, int y) {
        return fastMap[mapBounds.y * y + x];
    }
    cv::Point2i getBounds() {
        return this->mapBounds;
    }
};

#endif //MR_CPP_CODE_FASTMAP_H
