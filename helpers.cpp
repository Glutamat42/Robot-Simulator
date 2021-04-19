//
// Created by markus on 17.04.21.
//

#include "helpers.h"
#include "interactive_objects/interfaces/CollidableCircle.h"

double get_random_percentage(double in_range) {
    return fmod(rand() / 1000.0, in_range) - in_range / 2;
}

std::optional<cv::Point2d> collision_detection_circle_circle(CollidableCircle *c1, CollidableCircle *c2, cv::Point2d *pos) {
    if (!pos) {
        cv::Point2d p = c1->get_position();
        pos = &p;
    }

    double squared_distance =
            pow(pos->x - c2->get_position().x, 2) + pow(pos->y - c2->get_position().y, 2);
    double squared_radii = (c1->get_radius() + c1->get_radius()) * (c2->get_radius() + c2->get_radius());

    if (squared_radii > squared_distance) {
        // TODO: does not handle overlapping circles
        cv::Point2d estimated_collision_point = c1->get_position() + (c1->get_position() - c2->get_position()) / 2;
        return estimated_collision_point;
    }
    return {};
}

std::vector<cv::Point2d> collision_detection_ray_circle(CollidableRay *ray, CollidableCircle *circle) {
    cv::Point2d x1 = ray->get_position() - circle->get_position();
    cv::Point2d x2 = ray->getEndPoint() - circle->get_position();

    cv::Point2d dv = x2 - x1;
    double dr = sqrt(dv.x * dv.x + dv.y * dv.y);
    double D = x1.x * x2.y - x2.x * x1.y;

    // evaluate if there is an intersection
    double di = circle->get_radius() * circle->get_radius() * dr * dr - D * D;
    if (di < 0.0)
        return {};

    double t = sqrt(di);

    std::vector<cv::Point2d> ip;
    ip.push_back(cv::Point2d(D * dv.y + copysign(1.0, dv.y) * dv.x * t, -D * dv.x + abs(dv.y) * t) / (dr * dr) +
                 (circle->get_position()));
    if (di > 0.0) {
        ip.push_back(cv::Point2d(D * dv.y - copysign(1.0, dv.y) * dv.x * t, -D * dv.x - abs(dv.y) * t) / (dr * dr) +
                     (circle->get_position()));
    }
    return ip;
}
