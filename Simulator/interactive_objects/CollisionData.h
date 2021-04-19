//
// Created by markus on 19.04.21.
//

#ifndef MR_CPP_CODE_COLLISIONDATA_H
#define MR_CPP_CODE_COLLISIONDATA_H

#include "interfaces/CollidableObject.h"

class CollisionData {
private:
    CollidableObject *object;
    cv::Point2d point;
    double distance;
public:
    /**
     *
     * @param object the target object this one collided with
     * @param point the exact point where the collision happened
     * @param distance additional info, its meaning might differ, it could be eg the distance between point and the starting point of a vector
     */
    CollisionData(CollidableObject *object, cv::Point2d point, double distance = -1) {
        this->object = object;
        this->point = point;
        this->distance = distance;
    }

    CollidableObject* getObject() {
        return this->object;
    }

    cv::Point2d getPoint() {
        return this->point;
    }
};

#endif //MR_CPP_CODE_COLLISIONDATA_H
