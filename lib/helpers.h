//
// Created by markus on 17.04.21.
//

#ifndef MR_CPP_CODE_HELPERS_H
#define MR_CPP_CODE_HELPERS_H

#include <opencv2/imgproc.hpp>
#include "../Simulator/interactive_objects/interfaces/CollidableCircle.h"
#include "../Simulator/interactive_objects/interfaces/CollidableRay.h"
#include "FastMap.h"


/** check collision between two circles
 *
 * @param c1
 * @param c2
 * @return true if collision
 */
std::optional<cv::Point2d> collision_detection_circle_circle(CollidableCircle *c1, CollidableCircle *c2, cv::Point2d *pos = nullptr);

std::vector<cv::Point2d> collision_detection_ray_circle(CollidableRay *ray, CollidableCircle *circle, bool limitToLength = true) ;

bool pointBetween(cv::Point2d p1, cv::Point2d p2, cv::Point2d px);

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

/** get color from red (0) to green (1)
 *
 * @param power value between 0 and 1
 */
cv::Scalar getColor(double power);

std::array<double, 3> weightedAverageParticle(std::vector<std::array<double, 3>> particles, std::vector<double> weights);

FastMap padObstacles(FastMap map, double radius);

#endif //MR_CPP_CODE_HELPERS_H
