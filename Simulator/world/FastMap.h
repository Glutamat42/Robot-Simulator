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
        this->mapBounds = cv::Point2i(x, y);
        this->fastMap = std::vector<bool>(this->mapBounds.x * this->mapBounds.y, initialValue);
    }

    FastMap(cv::Mat grayscaleImage) {
        // accept only char type matrices
        CV_Assert(grayscaleImage.depth() == CV_8U);

        this->mapBounds = cv::Point2i(grayscaleImage.cols, grayscaleImage.rows);
        this->fastMap = std::vector<bool>(this->mapBounds.x * this->mapBounds.y);
        for (int y = 0; y < this->mapBounds.y; y++) {
            for (int x = 0; x < this->mapBounds.x; x++) {
                this->setPixel(x, y, grayscaleImage.at<uchar>(y, x) != 0);
            }
        }
    }

    void setPixel(int x, int y, bool value) {
        fastMap[mapBounds.y * y + x] = value;
    }

    bool getPixel(int x, int y) {
        return fastMap[mapBounds.y * y + x];
    }

    cv::Point2i getBounds() {
        return this->mapBounds;
    }

    cv::Mat toCVMat() {
        cv::Mat image(mapBounds.y, mapBounds.x, CV_8UC1);
        for (int x = 0; x < mapBounds.x; ++x) {
            for (int y = 0; y < mapBounds.y; ++y) {
                image.at<uchar>(y, x) = this->getPixel(x, y) * 255;
            }
        }
        return image;
    }

    FastMap getDownScaledMap(int factor) {
        FastMap scaledMap(mapBounds.x / factor, mapBounds.y / factor);

        for(int x = 0; x < mapBounds.x; x+=factor) {
            for(int y = 0; y < mapBounds.y; y+=factor) {
                bool setTrue = false;
                for(int factorX = 0; !setTrue && factorX < factor; ++factorX) {
                    for(int factorY = 0; !setTrue && factorY < factor; ++factorY) {
                        if (x+factorX >= mapBounds.x || y+factorY >= mapBounds.y) continue;
                        if (getPixel(x+factorX, y+factorY)) {
                            setTrue = true;
                        }
                    }
                }
                if (setTrue) scaledMap.setPixel(x/factor, y/factor, true);
            }
        }

        return scaledMap;
    }
};

#endif //MR_CPP_CODE_FASTMAP_H
