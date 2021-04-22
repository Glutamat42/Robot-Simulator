//
// Created by markus on 18.04.21.
//

#ifndef MR_CPP_CODE_WALLPOINT_H
#define MR_CPP_CODE_WALLPOINT_H

#include "interfaces/CollidablePoint.h"

/** At least currently this is more or less a dummy class to make grayscaleMap collisions compatible with the CollidableObjects system
 *
 */
class WallPoint : public CollidablePoint {
public:
    WallPoint(cv::Point2d pos);

    void handleCollision(CollidableObject* object) {
        std::cout << "Wall: Collision detected" << std::endl;
    }
};

#endif //MR_CPP_CODE_WALLPOINT_H
