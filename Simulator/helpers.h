//
// Created by markus on 17.04.21.
//

#ifndef MR_CPP_CODE_HELPERS_H
#define MR_CPP_CODE_HELPERS_H

#include "interactive_objects/interfaces/CollidableCircle.h"
#include "interactive_objects/interfaces/CollidableRay.h"


/** check collision between two circles
 *
 * @param c1
 * @param c2
 * @return true if collision
 */
std::optional<cv::Point2d> collision_detection_circle_circle(CollidableCircle *c1, CollidableCircle *c2, cv::Point2d *pos = nullptr);

std::vector<cv::Point2d> collision_detection_ray_circle(CollidableRay *ray, CollidableCircle *circle, bool limitToLength = true) ;

bool pointBetween(cv::Point2d p1, cv::Point2d p2, cv::Point2d px);

#endif //MR_CPP_CODE_HELPERS_H
