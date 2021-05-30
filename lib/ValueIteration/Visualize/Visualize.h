//
// Created by markus on 19.05.21.
//

#ifndef MR_CPP_CODE_VISUALIZE_H
#define MR_CPP_CODE_VISUALIZE_H

#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "constants.h"
#include "MapField.h"
#include "Colors.h"


class Visualize {
private:
    std::vector<std::vector<MapField>> matrix;
    cv::Point2i matrixBounds;
    std::vector<std::string> primaryLabels;
    std::vector<std::string> secondaryLabels;
public:
    std::string windowTitle = "PathFindingAlgo";

    /**
     * @param size shape of elements (columns/steps, processes)
     */
    explicit Visualize(cv::Point2i size);

    Visualize() : Visualize(cv::Point2i(0,0)) {}

    void setCell(cv::Point2i pos, MapField cell);

    MapField getCell(cv::Point2i pos);

    void setPrimaryLabels(std::vector<std::string> labels);

    void setSecondaryLabels(std::vector<std::string> labels);

    /** add or remove columns from/to current object
     *
     * @param n number of new columns; negative to remove columns
     */
    void resizeColumns(int n=1);

    cv::Point2i getMatrixBounds();

    /** render */
    void show(bool waitForKey = false);
};


#endif //MR_CPP_CODE_VISUALIZE_H
