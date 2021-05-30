//
// Created by markus on 30.05.21.
//

#ifndef MR_CPP_CODE_VALUEITERATIONDATASTRUCTURES_H
#define MR_CPP_CODE_VALUEITERATIONDATASTRUCTURES_H

struct ValueIterationElement {
    std::vector<double> V = {0};
    std::vector<int> bestDirection = {-1};
    double cellReward = 0;
    bool validField = false;
};

#endif //MR_CPP_CODE_VALUEITERATIONDATASTRUCTURES_H
