//
// Created by markus on 17.04.21.
//

#ifndef MR_CPP_CODE_HELPERS_H
#define MR_CPP_CODE_HELPERS_H

#include "interactive_objects/interfaces/CollidableCircle.h"

double get_random_percentage(double in_range = 0.025);


/** check collision between two circles
 *
 * @param c1
 * @param c2
 * @return true if collision
 */
bool collision_detection_circle_circle(CollidableCircle *c1, CollidableCircle *c2, cv::Point2d* pos = nullptr);


//bool collision_detection_ray_circle(CollidableRay *ray, CollidableCircle *circle);

#endif //MR_CPP_CODE_HELPERS_H
