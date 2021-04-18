//
// Created by markus on 17.04.21.
//

#include "helpers.h"
#include "interactive_objects/interfaces/CollidableCircle.h"

double get_random_percentage(double in_range) {
    return fmod(rand() / 1000.0, in_range) - in_range / 2;
}

bool collision_detection_circle_circle(CollidableCircle *c1, CollidableCircle *c2, cv::Point2d* pos) {
    if (!pos) {
        cv::Point2d p = c1->get_position();
        pos = &p;
    }

    double squared_distance =
            pow(pos->x - c2->get_position().x, 2) + pow(pos->y - c2->get_position().y, 2);
    double squared_radii = (c1->get_radius() + c1->get_radius()) * (c2->get_radius() + c2->get_radius());

    return squared_radii > squared_distance;
}