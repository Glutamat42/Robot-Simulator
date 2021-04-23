//
// Created by markus on 20.04.21.
//

#ifndef MR_CPP_CODE_PARTICLEOPERATOR_H
#define MR_CPP_CODE_PARTICLEOPERATOR_H


#include "RobotOperator.h"
#include "../Simulator/world/MapLine.h"
#include "../lib/ParticleFilter.h"

class ParticleOperator : public RobotOperator {
private:
    ParticleFilter particleFilter;
    RobotOperator *secondOperator;
public:
    ParticleOperator(RobotControlInterface *robot, std::string map_filename, bool benchmarkMode = false);

    void update() override;
};


#endif //MR_CPP_CODE_PARTICLEOPERATOR_H
