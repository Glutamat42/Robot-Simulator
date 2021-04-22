//
// Created by markus on 18.04.21.
//

#ifndef MR_CPP_CODE_COLLIDABLEPOINT_H
#define MR_CPP_CODE_COLLIDABLEPOINT_H

#include <opencv2/core/types.hpp>
#include <iostream>
#include "CollidableObject.h"

/** THIS CLASS IS NOT FULLY IMPLEMENTED AND NOT FULLY SUPPORTED ELSEWHERE
 *
 */
class CollidablePoint : public CollidableObject {
protected:
    cv::Point2d pos;

    CollidableObject * collision_detection_map(cv::Point2d* pos) {
        std::cout << "Currently not implemented because the point object is only used for non interactive walls" << std::endl;
        return nullptr;
    };

    std::vector<CollisionData *> collision_detection_objects(std::vector<CollidableObject *> collidableObjects, cv::Point2d* pos = nullptr) {
        std::cout << "Currently not implemented because the point object is only used for non interactive walls" << std::endl;
        return std::vector<CollisionData *>();
    }

public:
    CollidablePoint(World* world) : CollidableObject(world) {
        this->objectType = CollidableObjectType::point;
    }

    [[maybe_unused]] virtual cv::Point2d get_position() {
        return this->pos;
    };
};

#endif //MR_CPP_CODE_COLLIDABLEPOINT_H
