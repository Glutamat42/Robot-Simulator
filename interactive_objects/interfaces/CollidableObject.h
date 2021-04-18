//
// Created by markus on 16.04.21.
//

#ifndef MR_CPP_CODE_COLLIDABLEOBJECT_H
#define MR_CPP_CODE_COLLIDABLEOBJECT_H

#include <vector>

class World;

enum class CollidableObjectType {
    circle, point
};

class CollidableObject {
protected:
    CollidableObjectType objectType;

    virtual CollidableObject * collision_detection_map(cv::Point2d pos) = 0;

    virtual std::vector<CollidableObject *> collision_detection_objects(std::vector<CollidableObject *> collidableObjects, cv::Point2d* pos = nullptr) = 0;

    World *world;
public:
//    virtual ~CollidableObject() = 0;

    CollidableObjectType getObjectType() {
        return this->objectType;
    }

    virtual void handleCollision(CollidableObject *object) = 0;
};

//CollidableObject::~CollidableObject() {}

#endif //MR_CPP_CODE_COLLIDABLEOBJECT_H
