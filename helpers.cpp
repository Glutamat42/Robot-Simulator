//
// Created by markus on 15.04.21.
//

#include <cstdlib>
#include <cmath>

double get_random_percentage(double in_range=0.025) {
    return fmod(rand() / 1000.0, in_range) - in_range / 2;
}
