//
// Created by markus on 18.04.21.
//

#include "CollidableRay.h"
#include "../../constants.h"
#include "../WallPoint.h"
#include "../../world/world.h"
#include "../../../lib/helpers.h"
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
        cv::Point2d point = cv::Point2d(pos->x + i * dx, pos->y + i * dy);

        if (this->world->check_collision(point)) {
            *distance = i;
            return (new WallPoint(this->world, point));
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
                    int index = 0;
                    if (collisions.size() == 2) {
                        cv::Point2d diff0 = collisions[0] - *pos;
                        cv::Point2d diff1 = collisions[1] - *pos;
                        double d0 = diff0.x * diff0.x + diff0.y * diff0.y;
                        double d1 = diff1.x * diff1.x + diff1.y * diff1.y;
                        if (d1 < d0) {
                            index = 1;
                        }
                    }
                    collided_with.push_back(new CollisionData(object, collisions.at(index)));
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
}

bool CollidableRay::collision_detection_map_bool(cv::Point2d *pos, double *distance) {
//    auto *collisionObject = this->collision_detection_map(nullptr, distance);
//    bool collisionHappened = collisionObject != nullptr;
//    delete collisionObject;
//    return collisionHappened;

    // yep, this is duplicate code, but it is called very often and this variant is a little bit more efficient if the point of the collision is not required
    if (!pos) {
        pos = &this->pos;
    }

    double dx = cos(this->angle);
    double dy = sin(this->angle);

    for (double i = 0; i < this->length; i += CALCULATION_RESOLUTION) {
        cv::Point2d point = cv::Point2d(pos->x + i * dx, pos->y + i * dy);

        if (this->world->check_collision(point)) {
            *distance = i;
            return true;
        }
    }
    return false;
}

bool CollidableRay::collision_detection_objects_bool(std::vector<CollidableObject *> collidableObjects, cv::Point2d *pos) {
    std::vector<CollisionData *> collidedObjects = this->collision_detection_objects(collidableObjects, pos);
    bool collisionHappened = !collidedObjects.empty();
    for (auto d : collidedObjects) delete d;
    return collisionHappened;
}
