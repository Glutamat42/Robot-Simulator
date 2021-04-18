//
// Created by markus on 16.04.21.
//

#ifndef MR_CPP_CODE_COLLIDABLECIRCLE_H
#define MR_CPP_CODE_COLLIDABLECIRCLE_H

#include <opencv2/core/types.hpp>
#include "CollidableObject.h"
#include "../WallPoint.h"

class CollidableCircle : public CollidableObject {
protected:
    WallPoint * collision_detection_map(cv::Point2d pos);

    std::vector<CollidableObject *> collision_detection_objects(std::vector<CollidableObject *> collidableObjects, cv::Point2d* pos = nullptr);

    double radius;

    cv::Point2d pos;

public:
    CollidableCircle() {
        this->objectType = CollidableObjectType::circle;
    }

    double get_radius() {
        return this->radius;
    };

    cv::Point2d get_position() {
        return this->pos;
    };
};

#endif //MR_CPP_CODE_COLLIDABLECIRCLE_H
