//
// Created by markus on 22.04.21.
//

#ifndef MR_CPP_CODE_MAPOBJECT_H
#define MR_CPP_CODE_MAPOBJECT_H

class MapObject {
public:
    virtual void draw(cv::Mat image) = 0;
};

#endif //MR_CPP_CODE_MAPOBJECT_H
