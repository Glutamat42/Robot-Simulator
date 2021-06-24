//
// Created by markus on 21.06.21.
//

#ifndef MR_CPP_CODE_QLEARNINGSCENARIO_H
#define MR_CPP_CODE_QLEARNINGSCENARIO_H


#include "SimulationScenario.h"
#include "../Operators/QLearningOperator.h"

class QLearningScenario : public SimulationScenario  {
private:
    int NR_EPISODES = 1000;
    int MAX_STEPS_PER_EPISODE = 1000;

    int cur_step = 0;
    int cur_epoch = 0;
    std::map<std::vector<int>, std::map<std::array<int, 2>, double>> qMap;
    QLearningOperator* ro;
    std::ofstream logfile;

    void resetScenario();
    ~QLearningScenario();
public:
    void setUp() override;
    void afterUpdate() override;
};


#endif //MR_CPP_CODE_QLEARNINGSCENARIO_H
