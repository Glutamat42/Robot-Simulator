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

/**
 *
 * @param ray
 * @param circle
 * @param limitToLength if true: will only return points between start end endpoint
 * @return
 */
std::vector<cv::Point2d> collision_detection_ray_circle(CollidableRay *ray, CollidableCircle *circle, bool limitToLength) {
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

    if (limitToLength) {
        std::vector<cv::Point2d> ip_between_points;

        for (cv::Point2d point : ip) {
            if (pointBetween(ray->get_position(),ray->getEndPoint(), point)) {
                ip_between_points.push_back(point);
            }
        }

        ip = ip_between_points;
    }

    return ip;
}

bool pointBetween(cv::Point2d p1, cv::Point2d p2, cv::Point2d px) {
    cv::Point2d vec_p1_p2 = p1 - p2;
    double mag_p1_p2 = vec_p1_p2.x * vec_p1_p2.x + vec_p1_p2.y * vec_p1_p2.y;

    cv::Point2d vec_p1_px = p1 - px;
    double mag_p1_px = vec_p1_px.x * vec_p1_px.x + vec_p1_px.y * vec_p1_px.y;

    cv::Point2d vec_p2_px = p2 - px;
    double mag_p2_px = vec_p2_px.x * vec_p2_px.x + vec_p2_px.y * vec_p2_px.y;

    return (mag_p1_px < mag_p1_p2 && mag_p2_px < mag_p1_p2);
}
