//
// Created by markus on 18.04.21.
//

#include "CollidableRay.h"
#include "../../constants.h"
#include "../WallPoint.h"
#include "../../world.h"
#include "../../helpers.h"
#include "../CollisionData.h"

CollidableObject *CollidableRay::collision_detection_map(cv::Point2d *pos) {
    return collision_detection_map(nullptr, nullptr);
}

CollidableObject *CollidableRay::collision_detection_map(cv::Point2d *pos, double *distance) {
    if (!pos) {
        pos = &this->pos;
    }

    double dx = cos(this->angle);
    double dy = sin(this->angle);

    for (double i = 0; i < this->length; i += CALCULATION_RESOLUTION) {
        double ray_x = pos->x + i * dx;
        double ray_y = pos->y + i * dy;
        cv::Point2d point = cv::Point2d(ray_x, ray_y);

        if (this->world->check_collision(point)) {
            *distance = i;
            return (new WallPoint(point));
        }
    }
    return nullptr;
}

std::vector<CollisionData *>
CollidableRay::collision_detection_objects(std::vector<CollidableObject *> collidableObjects, cv::Point2d *pos) {
    // TODO: re-add pos support
    std::vector<CollisionData *> collided_with;

    if (!pos) {
        pos = &this->pos;
    }

    for (CollidableObject *object : collidableObjects) {
        // filter "this"
        if (this == object) continue;

        switch (object->getObjectType()) {
            case CollidableObjectType::circle: {
                std::vector<cv::Point2d> collisions = collision_detection_ray_circle(this,
                                                                                     static_cast<CollidableCircle *>(object));
                if (!collisions.empty()) {
//                    if (collisions.size() == 2) {
//
//                    }
                    collided_with.push_back(new CollisionData(object, collisions.at(1)));
                }
                break;
            }
            default: {
                std::cout << "Not implemented" << std::endl;
                break;
            }
        }
    }

    return collided_with;
};