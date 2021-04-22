//
// Created by markus on 18.04.21.
//

#include "WallPoint.h"

WallPoint::WallPoint(World *world, cv::Point2d pos) : CollidablePoint(world) {
    this->pos = pos;
}
