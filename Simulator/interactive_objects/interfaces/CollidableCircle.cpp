//
// Created by markus on 18.04.21.
//

#include "CollidableCircle.h"
#include "../../world.h"
#include "../../helpers.h"
#include "../CollisionData.h"

/** check for collision robot <-> wall
 *
 * @param pos center
 * @return position of collision (wall point); empty/null if no collision
 */
WallPoint *CollidableCircle::collision_detection_map(cv::Point2d *pos) {
    // check collisions on map
    for (int x = floor(pos->x) - this->radius; x <= ceil(pos->x) + radius; ++x) {
        for (int y = floor(pos->y) - this->radius; y <= ceil(pos->y) + radius; ++y) {
            if ((x - pos->x) * (x - pos->x) + (y - pos->y) * (y - pos->y) <= this->radius * this->radius) {
                cv::Point2d curPoint = cv::Point2d(x, y);
                if (this->world->check_collision(curPoint)) {
                    return new WallPoint(curPoint);
                }
            }
        }
    }
    return nullptr;
}

/** check collision with given objects list. Will return a list of all involved objects (excluding "this")
 *
 * @param collidableObjects
 * @return
 */
std::vector<CollisionData *>
CollidableCircle::collision_detection_objects(std::vector<CollidableObject *> collidableObjects, cv::Point2d *pos) {
    std::vector<CollisionData *> collided_with;

    if (!pos) {
        pos = &this->pos;
    }

    for (CollidableObject *object : collidableObjects) {
        // filter "this"
        if (this == object) continue;

        switch (object->getObjectType()) {
            case CollidableObjectType::circle: {
                std::optional<cv::Point2d> collision = collision_detection_circle_circle(this,
                                                                                         static_cast<CollidableCircle *>(object),
                                                                                         pos);
                if (collision) {
//                    std::cout << "Collision with circle detected" << std::endl;
                    collided_with.push_back(new CollisionData(this, collision.value()));
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