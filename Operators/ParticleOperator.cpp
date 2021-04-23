//
// Created by markus on 20.04.21.
//

#include <random>
#include "ParticleOperator.h"
#include "BasicWithNovelty.h"
#include "../lib/helpers.h"
#include <boost/math/distributions/exponential.hpp>
#include <chrono>

using namespace std::chrono;


ParticleOperator::ParticleOperator(RobotControlInterface *robot, std::string map_filename, bool benchmarkMode) : RobotOperator(robot), particleFilter(robot, map_filename, benchmarkMode) {
    this->secondOperator = new BasicWithNovelty(robot);
}


/** loop function which will be called once every tick
 *
 */
void ParticleOperator::update() {
    // using another operator for the actual movement, so this one can focus on particle filter
    this->secondOperator->update();
    this->particleFilter.update();

}
