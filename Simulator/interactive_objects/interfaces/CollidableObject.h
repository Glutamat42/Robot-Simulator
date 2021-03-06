//
// Created by markus on 16.04.21.
//

#ifndef MR_CPP_CODE_COLLIDABLEOBJECT_H
#define MR_CPP_CODE_COLLIDABLEOBJECT_H

#include <vector>

class CollisionData;

class World;

enum class CollidableObjectType {
    circle, point, ray
};

class CollidableObject {
protected:
    World *world;
    CollidableObjectType objectType;

    virtual CollidableObject *collision_detection_map(cv::Point2d *pos = nullptr) = 0;

    virtual std::vector<CollisionData *> collision_detection_objects(std::vector<CollidableObject *> collidableObjects, cv::Point2d *pos = nullptr) = 0;
public:
    CollidableObject(World* world) {
        this->world = world;
    }

    virtual ~CollidableObject() {};

    CollidableObjectType getObjectType() {
        return this->objectType;
    }

    virtual void handleCollision(CollidableObject *object) = 0;
};


#endif //MR_CPP_CODE_COLLIDABLEOBJECT_H
