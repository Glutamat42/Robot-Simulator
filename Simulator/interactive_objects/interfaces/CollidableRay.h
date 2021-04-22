//
// Created by markus on 18.04.21.
//

#ifndef MR_CPP_CODE_COLLIDABLERAY_H
#define MR_CPP_CODE_COLLIDABLERAY_H

#include <opencv2/core/types.hpp>
#include <iostream>
#include "CollidableObject.h"

class CollidableRay : public CollidableObject {
protected:
    /** start position */
    cv::Point2d pos;
    double angle;
    double length;

    CollidableObject *collision_detection_map(cv::Point2d *pos = nullptr) override;

    /** Calculate point of collision and return distance over distance pointer
     *
     * @param pos
     * @param distance additional return value: Distance from sensor to collision
     * @return
     */
    CollidableObject *collision_detection_map(cv::Point2d *pos = nullptr, double *distance = nullptr);

    // TODO: implement for all CollidableObjects
    bool collision_detection_map_bool(cv::Point2d* pos = nullptr, double* distance = nullptr);

    std::vector<CollisionData *> collision_detection_objects(std::vector<CollidableObject *> collidableObjects, cv::Point2d *pos = nullptr);

    // TODO: implement for all CollidableObjects
    bool collision_detection_objects_bool(std::vector<CollidableObject *> collidableObjects, cv::Point2d *pos = nullptr);

public:
    CollidableRay(World* world, double angle, double length = 800) : CollidableObject(world) {
        this->angle = angle;
        this->length = length;
        this->objectType = CollidableObjectType::ray;
    }

    virtual cv::Point2d get_position() {
        return this->pos;
    };

    double get_length() const {
        return this->length;
    }

    cv::Point2d getEndPoint() const {
        return cv::Point2d(
                pos.x + length * cos(this->angle),
                pos.y + length * sin(this->angle)
        );
    }
};

#endif //MR_CPP_CODE_COLLIDABLERAY_H
