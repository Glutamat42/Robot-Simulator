//
// Created by markus on 20.04.21.
//

#include <random>
#include "ParticleOperator.h"
#include "BasicWithNovelty.h"
#include "../Simulator/helpers.h"
#include <boost/math/distributions/exponential.hpp>
#include <chrono>

using namespace std::chrono;

int INIT_N = 5000;
int TARGET_N = 100;
int TARGET_SHOULD_BE_REACHED_AFTER_ITERS = 200;

ParticleOperator::ParticleOperator(RobotControlInterface *robot, std::string map_filename) : RobotOperator(robot) {
    this->particles_world = new World(map_filename, "Particle filter");
    this->secondOperator = new BasicWithNovelty(robot);

    this->N = INIT_N;

    this->particles = create_uniform_particles(this->particles_world->get_map_bounds().x,
                                               this->particles_world->get_map_bounds().y,
                                               this->N);

    this->updateParticleSimulation(this->particles, true);
}

/** create a list of N particles with uniformly distributed positions over the grayscaleMap (x_range, y_range) and a normaly
 * distributed angle
 */
std::vector<std::array<double, 3>> ParticleOperator::create_uniform_particles(int x_range, int y_range, int N) {
    std::vector<std::array<double, 3>> particles;
    std::default_random_engine generator;
    generator.seed(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());

    std::uniform_real_distribution<double> x_distribution(0, x_range);
    std::uniform_real_distribution<double> y_distribution(0, y_range);
    std::uniform_real_distribution<double> angle_distribution(-M_PI, M_PI);

    // TESTING
    // exact match
//    particles.push_back(std::array<double, 3>({320,
//                                               40,
//                                               1.18*M_PI}));
    // good match
//    particles.push_back(std::array<double, 3>({315,
//                                               45,
//                                               1.17*M_PI}));
    // END TESTING

    for (int i = 0; i < N; i++) {
        particles.push_back(std::array<double, 3>({x_distribution(generator),
                                                   y_distribution(generator),
                                                   angle_distribution(generator)}));
    }
    return particles;
}

/** do prediction
 * update particle position & angle based on the (estimated) moved distance and angle
 */
std::vector<std::array<double, 3>> ParticleOperator::particles_predict(std::vector<std::array<double, 3>> *oldParticles,
                                                                       double move_distance,
                                                                       double move_angle,
                                                                       double standard_deviation) {
    std::vector<std::array<double, 3>> updated_particles;

    // init normal_distribution
    std::random_device rd{};
    std::mt19937 gen{rd()};
    std::normal_distribution<> d{0, standard_deviation};

    // update oldParticles
    for (std::array<double, 3> particle : *oldParticles) {
        // update moved distance and angle with a norm distribution value (x * ((100 + <norm_dist>) / 100))
        double distance_with_distribution = move_distance * ((100 + d(gen)) / 100);
        double angle_with_distribution = move_angle * ((100 + d(gen)) / 100);

        double newRobotAngle = angle_with_distribution + particle[2];

        // update oldParticles with distance and angle
        updated_particles.push_back(std::array<double, 3>(
                {
                        particle[0] + cos(newRobotAngle) * distance_with_distribution,
                        particle[1] + sin(newRobotAngle) * distance_with_distribution,
                        newRobotAngle
                }));
    }

    return updated_particles;
}


void ParticleOperator::updateParticleSimulation(std::vector<std::array<double, 3>> particles, bool showMap, std::vector<double> weights) {
    // get sensor config to recreate (clone) the same sensors as on the robot
    std::vector<std::array<double, 2>> sensor_config;
    for (DistanceSensor *sensor : this->filter_for_distance_sensor(this->robot->get_sensors())) {
        sensor_config.push_back(std::array<double, 2>({sensor->get_sensor_angle(), sensor->get_sensor_max_distance()}));
    }

    double maxWeight = 0;
    double useCustomWeightColors = false;
    if (!weights.empty()) {
        useCustomWeightColors = true;
        for (double weight : weights) {
            maxWeight = maxWeight > weight ? maxWeight : weight;
        }
    }

    // create dummy robots representing the particles
    this->particles_world->clearObjectsList();
    for (int i = 0; i < particles.size(); ++i) {
        std::array<double, 3> particle = particles[i];
        // TODO: creating a dummyRobot class would allow updating existing robots. This would allow to avoid recreating everything all the times
        Robot *dummyRobot = new Robot("particle",
                                      this->robot->get_radius(),
                                      cv::Point2d(particle[0], particle[1]),
                                      particle[2]);
        // create sensors
        for (std::array<double, 2> s_config : sensor_config) {
            DistanceSensor *sensor = new DistanceSensor(this->particles_world, dummyRobot, s_config[0], s_config[1], true);
            sensor->update_sensor_data(true);
            dummyRobot->add_sensor(sensor);
        }

        // set custom visualization options
        if (useCustomWeightColors) {
            dummyRobot->setDrawOptions(getColor(weights[i] / maxWeight), true, 1);
        } else {
            dummyRobot->setDrawOptions(CV_RGB(255, 0, 0), true, 1);
        }

        // add robot to simulation world
        this->particles_world->add_object(dummyRobot);
    }

    if (showMap) {
        this->particles_world->show_map(true);
    }
}


std::vector<double> ParticleOperator::particles_update(std::vector<double> robotSensorValues, double lambda) {
    std::vector<Robot *> simRobots = this->particles_world->get_robots();
    std::vector<double> updated_weights = std::vector<double>(simRobots.size(), 1.0);
    auto d = boost::math::exponential_distribution<>{lambda};

    double sumOfWeights = 0;

    std::cout << std::fixed;
    std::cout << std::setprecision(10);

    for (int i = 0; i < updated_weights.size(); ++i) {
        // collect sensor values of simulated particles
        std::vector<DistanceSensor *> sensors = this->filter_for_distance_sensor(simRobots.at(i)->get_sensors());
        std::vector<double> simulationSensorValues = {};
        for (DistanceSensor *sensor : sensors) {
            simulationSensorValues.push_back(sensor->get_simplified_sensor_value());
        }

//        std::cout << simRobots.at(i)->get_position().x << ";" << simRobots.at(i)->get_position().y << ";" << simRobots.at(i)->get_orientation() << ";";

        // calculate correlation
        for (int j = 0; j < simulationSensorValues.size(); ++j) {
            updated_weights[i] *= boost::math::pdf(d, abs(simulationSensorValues[j] - robotSensorValues[j]));
            updated_weights[i] += 1.e-300; // prevent rounding to 0

//            if (i == 0) std::cout << robotSensorValues[j]<<";";
//            else std::cout << simulationSensorValues[j] <<";";
        }

        sumOfWeights += updated_weights[i];

//        std::cout << updated_weights[i] << ";" <<std::endl;
    }

//    double maxWeight = 0;

    // sum of weights should equal 1
    for (int i = 0; i < updated_weights.size(); ++i) {
        updated_weights[i] /= sumOfWeights;
//        if (updated_weights[i] > maxWeight) {
//            maxWeight = updated_weights[i];
//        }
    }
//    std::cout << "max weight: " << maxWeight << std::endl;

    return updated_weights;
}

std::tuple<std::vector<std::array<double, 3>>, std::vector<double>>
ParticleOperator::particles_resample(std::vector<std::array<double, 3>> *oldParticles, std::vector<double> *weights, int N, double noise) {
    // resample algorithm is explained here: https://robotics.stackexchange.com/a/481
    std::vector<double> positions = {};
    for (int i = 0; i < N; ++i) {
//        positions.push_back(((double) rand() / RAND_MAX));
        positions.push_back((i + ((double) rand() / RAND_MAX)) / N);
    }

    std::vector<int> indexes(N, 0);
    std::vector<double> cumulativeSum = {};
    cumulativeSum.push_back(weights->at(0));
    for (int i = 1; i < N; ++i) {
        cumulativeSum.push_back(cumulativeSum.at(i - 1) + weights->at(i));
    }

    int i = 0;
    int j = 0;
    while (i < N && j < N) {
        if (positions[i] < cumulativeSum[j]) {
            indexes[i] = j;
            i += 1;
        } else {
            j += 1;
        }
    }


    std::default_random_engine generator;
    std::uniform_real_distribution<double> uniformNoise(-noise, noise);

    std::vector<std::array<double, 3>> updatedParticles = {};
    std::vector<double> updatedWeights = {};
    double weightSum = 0;
    for (int i = 0; i < N; ++i) {
        updatedParticles.push_back(std::array<double, 3>({oldParticles->at(indexes[i])[0] + uniformNoise(generator),
                                                          oldParticles->at(indexes[i])[1] + uniformNoise(generator),
                                                          oldParticles->at(indexes[i])[2] + uniformNoise(generator) / 10
                                                         }));
        updatedWeights.push_back(weights->at(indexes[i]));
        weightSum += weights->at(indexes[i]);
    }
    for (int i = 0; i < N; ++i) {
        weights->at(indexes[i]) /= weightSum;
    }


    // replace random particles with random values to allow recovery in case the detection went horribly wrong
    std::vector<std::array<double, 3>> randomParticles = create_uniform_particles(this->particles_world->get_map_bounds().x, this->particles_world->get_map_bounds().y, N/100);
    std::uniform_int_distribution<int> uniformIndexCreator(0, N);
    for (int i = 0; i < N/100 ; ++i) {
//        continue;
        updatedParticles[uniformIndexCreator(generator)] = randomParticles[i];
    }


    return std::tuple<std::vector<std::array<double, 3>>, std::vector<double>>(updatedParticles, updatedWeights);
}


void ParticleOperator::update() {
    if (iterationsCounter <= TARGET_SHOULD_BE_REACHED_AFTER_ITERS && iterationsCounter > 0) {
        int removeElementsCount = (INIT_N - TARGET_N) / TARGET_SHOULD_BE_REACHED_AFTER_ITERS;
        this->N -= removeElementsCount;

        bool finished = false;
        for (int i=0; !finished && i < this->particles.size(); ++i) {
            finished = true;
            for (int j=0; j < this->particles.size(); ++j) {
                if (this->weights[i] < this->weights[j]) {
                    finished = false;
                    double tmpWeight = this->weights[i];
                    std::array<double, 3> tmpParticle = this->particles[i];
                    this->weights[i] = this->weights[j];
                    this->weights[j] = tmpWeight;
                    this->particles[i] = this->particles[j];
                    this->particles[j] = tmpParticle;
                }
            }
        }

        this->weights.erase(this->weights.begin(), this->weights.begin() + removeElementsCount);
        this->particles.erase(this->particles.begin(), this->particles.begin() + removeElementsCount);
    }
    this->iterationsCounter++;
    // DEBUG PERF
//    if (iterationsCounter == 50) exit(0);
    // END DEBUG PERF

    this->secondOperator->update();

//    // accumulate movement of some steps
//    if (stepsCounter < GAME_TPS) { // once per second
//        this->stepsCounter++;
//        this->currentStepTurnedByAngle += this->robot->get_last_tick_movement_angle();
//        this->currentStepMoved += this->robot->get_last_tick_movement_distance();
//    } else {
//        this->stepsCounter = 0;
//        double angleTurned = this->robot->get this->currentStepTurnedByAngle
//    }

    std::vector<std::array<double, 3>> updated_particles = particles_predict(&this->particles,
                                                                             this->robot->get_last_tick_movement_distance(),
                                                                             this->robot->get_last_tick_movement_angle(),
                                                                             4);

    // generate vector containing the current sensor values of the "real" robot
    std::vector<double> robotSensorValues = {};
    for (DistanceSensor *sensor : this->filter_for_distance_sensor(this->robot->get_sensors())) {
        robotSensorValues.push_back(sensor->get_simplified_sensor_value());
    }

    // this allows to read the sensor values of all simulated particles in particles_update()
    this->updateParticleSimulation(updated_particles, false);

    std::vector<double> updated_weights = this->particles_update(robotSensorValues, 0.01);

    // nice for visualization but will impact performance!
    this->updateParticleSimulation(updated_particles, true, updated_weights);

    std::tuple<std::vector<std::array<double, 3>>, std::vector<double>> resampledTuple = this->particles_resample(&updated_particles,
                                                                                                                  &updated_weights,
                                                                                                                  this->N);
    this->particles = std::get<0>(resampledTuple);
    this->weights = std::get<1>(resampledTuple);
}
