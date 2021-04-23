//
// Created by markus on 22.04.21.
//

#ifndef MR_CPP_CODE_ASTAROPERATOR_H
#define MR_CPP_CODE_ASTAROPERATOR_H


#include "RobotOperator.h"

class AStarOperator : public RobotOperator {
private:
public:
    AStarOperator(RobotControlInterface *robot, std::string map_filename);

    void update() override;
};


#endif //MR_CPP_CODE_ASTAROPERATOR_H
