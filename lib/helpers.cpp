//
// Created by markus on 17.04.21.
//

#include "helpers.h"

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
            if (pointBetween(ray->get_position(), ray->getEndPoint(), point)) {
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

std::vector<cv::Scalar> colorLookupTable = {CV_RGB(255, 0, 0), CV_RGB(255, 128, 0), CV_RGB(255, 255, 0), CV_RGB(128, 255, 0), CV_RGB(0, 255, 0)};

cv::Scalar getColor(double power) {
    if (power > 1) {
        throw std::invalid_argument("power has to be between 0 and 1");
    }
    return colorLookupTable[(int) (power * colorLookupTable.size())];
}

/** calculate weighted average particle
 *
 * @param particles particles vector of arrays of format [x, y, angle]
 * @param weights normalized weights, same length as particles
 * @return
 */
std::array<double, 3> weightedAverageParticle(std::vector<std::array<double, 3>> particles, std::vector<double> weights) {
    int a = 1;
    double b1 = 0;
    double tx1 = 0;
    double ty1 = 0;
    double tangle1 = 0;
    for (int i = 0; i < particles.size(); ++i) {
        double weight = weights[i];
        std::array<double, 3> particle = particles[i];
        b1 += pow(1 - weight, a);
        tx1 += pow(particle[0] * (1 - weight), a);
        ty1 += pow(particle[1] * (1 - weight), a);
        tangle1 += pow(particle[2] * (1 - weight), a);
    }
    double b = 1 / b1;
    std::array<double, 3> result({b * tx1, b * ty1, b * tangle1});

    return result;
}

/** Add padding around obstacles. */
FastMap padObstacles(FastMap map, double radius) {
    cv::Point2i paddedMapBounds = map.getBounds();
    FastMap paddedMap = FastMap(paddedMapBounds.x, paddedMapBounds.y);
    FastMap kernel = FastMap((int)ceil(radius) * 2 + 1, (int)ceil(radius) * 2 + 1, false);
    cv::Point2i kernelBounds = kernel.getBounds();
    cv::Point2i center((int)ceil(radius), (int)ceil(radius)); // center is radius + 1, but we start counting at 0 so it is radius + 1 - 1 = radius

    // create circle kernel
    for (int x = center.x - radius; x <= center.x + radius; ++x) {
        for (int y = center.y - radius; y <= center.y + radius; ++y) {
            if ((x - center.x) * (x - center.x) + (y - center.y) * (y - center.y) <= radius * radius) {
                kernel.setPixel(x, y, true);
            }
        }
    }

    // apply kernel
    // iterate through all pixels
    for (int mapX = 0; mapX < paddedMapBounds.x; ++mapX) {
        for (int mapY = 0; mapY < paddedMapBounds.y; ++mapY) {
            // if current pixel is true the kernel has to be applied to the padded map
            if (map.getPixel(mapX, mapY)) {
                // iterate through all pixels of the kernel
                for (int kernelX = 0; kernelX < kernelBounds.x; ++kernelX) {
                    for (int kernelY = 0; kernelY < kernelBounds.y; ++kernelY) {
                        // if current kernel position is not true we dont have to do anything
                        if (kernel.getPixel(kernelX, kernelY)) {
                            int paddedX = mapX + kernelX - center.x - 1; // " - center.x - 1" because the kernel center has to be mapped to the current pixel (offset)
                            int paddedY = mapY + kernelY - center.y - 1;

                            // check boundaries
                            if (paddedX < 0 || paddedY < 0 || paddedX >= paddedMapBounds.x || paddedY >= paddedMapBounds.y) continue;

                            // all checks passed: set pixel on new map
                            paddedMap.setPixel(paddedX, paddedY, true);
                        }
                    }
                }
            }
        }
    }
    return paddedMap;
}


