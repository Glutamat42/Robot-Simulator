//
// Created by markus on 20.04.21.
//

#ifndef MR_CPP_CODE_PARTICLEOPERATOR_H
#define MR_CPP_CODE_PARTICLEOPERATOR_H


#include "RobotOperator.h"

class ParticleOperator : public RobotOperator {
private:
    World* particles_world;

//    bool update_turning = false;
//    bool update_moving = false;

    RobotOperator* secondOperator;

    void updateParticleSimulation(std::vector<std::array<double, 3>> particles, bool showMap);

    std::vector<std::array<double, 3>> particles;
    std::vector<double> weights;

    std::vector<std::array<double, 3>> create_uniform_particles(int x_range, int y_range, int N);
    std::vector<std::array<double, 3>> particles_predict(std::vector<std::array<double, 3>> *oldParticles, double move_distance, double move_angle, double standard_deviation = 1);
    std::vector<double> particles_update(std::vector<double> robotSensorValues, double lambda = 0.5);
    std::tuple<std::vector<std::array<double, 3>>, std::vector<double>> particles_resample(std::vector<std::array<double, 3>> *oldParticles, std::vector<double> *weights);
public:
    ParticleOperator (RobotControlInterface* robot, std::string map_filename);

    void update() override;
};


#endif //MR_CPP_CODE_PARTICLEOPERATOR_H
