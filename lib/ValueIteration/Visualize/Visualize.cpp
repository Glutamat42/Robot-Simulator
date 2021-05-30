//
// Created by markus on 19.05.21.
//

#include "Visualize.h"


Visualize::Visualize(cv::Point2i size) {
    this->matrix.resize(size.x, std::vector<MapField>(size.y, MapField()));
    this->matrixBounds = size;
}

void Visualize::show(bool waitForKey) {
    // size of table
    int fieldTotalWidth = (FIELD_WIDTH + 2 * BORDER_WIDTH + TABLE_BORDER_WIDTH);
    int fieldTotalHeight = (FIELD_HEIGHT + 2 * BORDER_WIDTH + TABLE_BORDER_WIDTH);
    int tableWidth = (fieldTotalWidth * this->matrixBounds.x) + TABLE_BORDER_WIDTH;
    int tableHeight = (fieldTotalHeight * this->matrixBounds.y) + TABLE_BORDER_WIDTH;

    // adjust size for labels
    int totalWidth = tableWidth;
    int totalHeight = tableHeight + BORDER_WIDTH + 23;  // "BORDER_WIDTH + x" for numbers row
    const int primLabelWidth = 2*FIELD_WIDTH;
    const int secLabelWidth = 2*FIELD_WIDTH;
    if (!this->primaryLabels.empty()) {
        totalWidth += primLabelWidth;
    }
    if (!this->secondaryLabels.empty()) {
        totalWidth += secLabelWidth;
    }

    // create blank image
    cv::Mat image = cv::Mat::zeros(totalHeight, totalWidth, CV_8UC3);

    // create background of table (cell borders)
    cv::rectangle(image, cv::Point2i(0,0), cv::Point2i(tableWidth, tableHeight), Colors::gray, cv::FILLED);

    // draw cells
    for (int x = 0; x < this->matrixBounds.x; ++x) {
        int x_offset = x * fieldTotalWidth;
        for (int y = 0; y < this->matrixBounds.y; ++y) {
            int y_offset = y * fieldTotalHeight;
            cv::Mat currentTile = this->matrix[x][y].draw();
            currentTile.copyTo(image(cv::Rect(x_offset + TABLE_BORDER_WIDTH, y_offset + TABLE_BORDER_WIDTH,currentTile.cols, currentTile.rows)));
        }
    }

    // labels
    int x_offset = tableWidth;
    if (!this->primaryLabels.empty()) {
        for (int y = 0; y < this->primaryLabels.size(); ++y) {
            cv::putText(image, this->primaryLabels[y], cv::Point2i(x_offset + BORDER_WIDTH*2, fieldTotalHeight * y + fieldTotalHeight - BORDER_WIDTH*2), cv::FONT_HERSHEY_SIMPLEX, 0.65, Colors::white, 2);
        }

        x_offset += primLabelWidth;  // adjust x_offset in case of more label columns (sec labels) will be added
    }
    if (!this->secondaryLabels.empty()) {
        for (int y = 0; y < this->secondaryLabels.size(); ++y) {
            cv::putText(image, this->secondaryLabels[y], cv::Point2i(x_offset + BORDER_WIDTH*2, fieldTotalHeight * y + fieldTotalHeight - BORDER_WIDTH*2), cv::FONT_HERSHEY_SIMPLEX, 0.5, Colors::white, 1);
        }
    }

    // numbers
    for (int x = 0; x < this->matrixBounds.x; ++x) {
        cv::putText(image, std::to_string(x), cv::Point2i(x*fieldTotalWidth+BORDER_WIDTH, totalHeight-BORDER_WIDTH), cv::FONT_HERSHEY_SIMPLEX, 0.65, Colors::white, 2);
    }


    cv::imshow(this->windowTitle, image);
    cv::waitKey();
}

void Visualize::setPrimaryLabels(std::vector<std::string> labels) {
    this->primaryLabels = labels;
}

void Visualize::setSecondaryLabels(std::vector<std::string> labels) {
    this->secondaryLabels = labels;
}

void Visualize::resizeColumns(int n) {
    this->matrixBounds.x += n;
    this->matrix.resize(this->matrixBounds.x, std::vector<MapField>(this->matrixBounds.y, MapField()));
}

cv::Point2i Visualize::getMatrixBounds() {
    return this->matrixBounds;
}

void Visualize::setCell(cv::Point2i pos, MapField cell) {
    this->matrix[pos.x][pos.y] = cell;
}

MapField Visualize::getCell(cv::Point2i pos) {
    return this->matrix[pos.x][pos.y];
}

