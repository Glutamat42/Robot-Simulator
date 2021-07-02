//
// Created by markus on 23.04.21.
//

#include <random>
#include <boost/math/distributions/exponential.hpp>
#include "ParticleFilter.h"
#include "../Simulator/world/world.h"
#include "../Simulator/interactive_objects/DistanceSensor.h"
#include "helpers.h"
#include "DummyRobot.h"

ParticleFilter::ParticleFilter(RobotControlInterface *robot, std::string map_filename, int initialN, int targetN) {
    this->particles_world = new World(map_filename, "Particle filter");
    this->particles_world->addMapObject(this->mapLine);
    this->robot = robot;

    this->N = initialN;
    this->TARGET_N = targetN;

    // create initial particles distribution and show the simulation map
    this->particles = ParticleFilter::create_uniform_particles(this->particles_world->get_map_bounds().x,
                                                               this->particles_world->get_map_bounds().y,
                                                               this->N);
    this->estimatedRobot = new MapRobot(robot->get_radius());
    this->estimatedRobot->setColor(CV_RGB(255, 255, 255));
    this->particles_world->addMapObject(this->estimatedRobot);
    ParticleFilter::updateParticleSimulation(this->particles, SHOW_WHATS_GOING_ON);
}

std::vector<std::array<double, 3>> ParticleFilter::create_uniform_particles(double x_range, double y_range, int N) {
    std::vector<std::array<double, 3>> particles;
    std::default_random_engine generator;
    generator.seed(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());

    std::uniform_real_distribution<double> x_distribution(0, x_range);
    std::uniform_real_distribution<double> y_distribution(0, y_range);
    std::uniform_real_distribution<double> angle_distribution(-M_PI, M_PI);

    for (int i = 0; i < N; i++) {
        particles.push_back(std::array<double, 3>({x_distribution(generator),
                                                   y_distribution(generator),
                                                   angle_distribution(generator)}));
    }
    return particles;
}

void ParticleFilter::updateParticleSimulation(std::vector<std::array<double, 3>> particles, bool showMap, std::vector<double> weights) {
    // get sensor config to recreate (clone) the same sensors as on the robot
    std::vector<std::array<double, 2>> sensor_config;
    for (DistanceSensor *sensor : DistanceSensor::filter_for_distance_sensor(this->robot->get_sensors())) {
        sensor_config.push_back(std::array<double, 2>({sensor->get_sensor_angle(), sensor->get_sensor_max_distance()}));
    }

    // If weights are provided they will be visualized on the map. For the color scaling we have to know what the maximum probability of one particle is
    double maxWeight = 0;
    bool useCustomWeightColors = false;
    if (!weights.empty()) {
        useCustomWeightColors = true;
        for (double weight : weights) {
            maxWeight = maxWeight > weight ? maxWeight : weight;
        }
    }


    // add or delete robots from the world if the amount does not match
    if (this->particles_world->getRobotsList().size() > particles.size()) {
        // there are too many particles -> remove some
        this->particles_world->deleteRobotByIndex(particles.size() - 1, this->particles_world->getRobotsList().size() - particles.size() + 1, true);
    } else if (this->particles_world->getRobotsList().size() < particles.size()) {
        // there arent enough particles -> add some
        for (long i = this->particles_world->getRobotsList().size(); i<particles.size(); ++i) {
            // create robot
            auto* dummyRobot = new DummyRobot("particle", this->robot->get_radius(), this->particles_world);
            // set default visualization options
            dummyRobot->setDrawOptions(CV_RGB(255, 0, 0), true, 1);
            // add robot to world
            this->particles_world->addRobot(dummyRobot);

            // create sensors
            for (std::array<double, 2> s_config : sensor_config) {
                DistanceSensor *sensor = new DistanceSensor(this->particles_world, dummyRobot, s_config[0], s_config[1], true);
                dummyRobot->add_sensor(sensor);
            }
        }
    }

    // update robots
    std::vector<Robot*> dummyRobotsList = this->particles_world->getRobotsList();
    for (int i = 0; i < particles.size(); ++i) {
        std::array<double, 3> particle = particles[i];
        DummyRobot* dummyRobot = (DummyRobot*) dummyRobotsList[i];
        dummyRobot->reposition(particle[0], particle[1], particle[2]);

        // set custom visualization options
        if (useCustomWeightColors) {
            dummyRobot->setDrawOptions(getColor(weights[i] / maxWeight), true, 1);
        }

        for (SensorInterface* sensor : dummyRobot->get_sensors()) {
            sensor->update_sensor_data(true);
        }
    }

    // show map if enabled
    if (showMap) {
        this->particles_world->show_map(true);
    }
}

std::vector<std::array<double, 3>> ParticleFilter::particles_predict(std::vector<std::array<double, 3>> *oldParticles,
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

std::vector<double> ParticleFilter::particles_update(std::vector<double> robotSensorValues, std::vector<Robot *> simRobots, double lambda) {
    std::vector<double> updated_weights = std::vector<double>(simRobots.size(), 1.0);
    auto d = boost::math::exponential_distribution<>{lambda};

    double sumOfWeights = 0;

    for (int i = 0; i < updated_weights.size(); ++i) {
        // collect sensor values of simulated particles
        std::vector<DistanceSensor *> sensors = DistanceSensor::filter_for_distance_sensor(simRobots.at(i)->get_sensors());
        int j = 0;
        for (DistanceSensor *sensor : sensors) {
            // calculate correlation
            updated_weights[i] *= boost::math::pdf(d, abs(sensor->get_simplified_sensor_value() - robotSensorValues[j]));
            updated_weights[i] += 1.e-300; // prevent rounding to 0
            j++;
        }

        sumOfWeights += updated_weights[i];
    }

    // normalize weights: sum of weights should equal 1
    for (double &updated_weight : updated_weights) {
        updated_weight /= sumOfWeights;
    }

    return updated_weights;
}

std::tuple<std::vector<std::array<double, 3>>, std::vector<double>> ParticleFilter::particles_resample(
        std::vector<std::array<double, 3>> *oldParticles,
        std::vector<double> *weights,
        int N,
        bool enableRandomParticles,
        double noise,
        cv::Point2d mapBounds) {
    // grouping algorithm for resampling is explained here: https://robotics.stackexchange.com/a/481
    // creates an index list of entries to keep
    std::vector<double> positions = {};
    for (int i = 0; i < N; ++i) {
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

    // create a new particles list. It will be based on the previous one. Based on the index list created above some values will be kept, others
    // will be deleted. There will be many duplicates, which is expected. Each value will be modified by a noise value.
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

    // normalize weights
    for (int i = 0; i < N; ++i) {
        weights->at(indexes[i]) /= weightSum;
    }

    return std::tuple<std::vector<std::array<double, 3>>, std::vector<double>>(updatedParticles, updatedWeights);
}


ParticleEvaluationData ParticleFilter::update() {
    return this->update(this->robot->get_last_tick_movement_distance(), this->robot->get_last_tick_movement_angle());
}


ParticleEvaluationData ParticleFilter::update(double distance, double angle) {
    // check if in location finding phase
    int removeElementsCount = N * INITIAL_PHASE_REPLACE_SHARE;;
    if (!this->initialLocationFinished && this->iterationsCounter > 0) {
        // check if location is found
        if (this->locationCertaintyEstimation > CERTAINTY_ESTIMATION_THRESHOLD) {
            // check uncertainty metric if location seems to be already successful
            std::cout << "Found location" << std::endl;
            this->initialLocationFinished = true;
            this->N = TARGET_N;
            this->useRandomParticles = false;
        }

        // if location not yet found: select the worst rated particles and replace them with random
        if (!this->initialLocationFinished) {
            // bubble sort
            // could be replaced with a more efficient algorithm if its computationally too expensive
            bool finished = false;
            for (int i = 0; !finished && i < this->particles.size(); ++i) {
                finished = true;
                for (int j = i + 1; j < this->particles.size(); ++j) {
                    if (this->weights[i] > this->weights[j]) {
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

            // drop the first particles (lowest probability)
            this->weights.erase(this->weights.begin(), this->weights.begin() + removeElementsCount);
            this->particles.erase(this->particles.begin(), this->particles.begin() + removeElementsCount);

            // generate new particles to replace the removed ones
            std::vector<std::array<double, 3>> randomParticles = create_uniform_particles(this->particles_world->get_map_bounds().x,
                                                                                          this->particles_world->get_map_bounds().y,
                                                                                          removeElementsCount);
            this->particles.insert(this->particles.end(), randomParticles.begin(), randomParticles.end());
        }
    }
    this->iterationsCounter++;


    // Begin of actual particle filter steps

    // Particle Filter Step 1) particle prediction: apply estimated movement distance / angle (currently in this simulator there is no noise for those values)
    std::vector<std::array<double, 3>> updated_particles = particles_predict(&this->particles,
                                                                             distance,
                                                                             angle,
                                                                             4);

    // Particle Filter Step 2) update weights: update the weight of each particle based on how much the simulated sensor values correspond to the ones of our "real" robot
    // generate vector containing the current sensor values of the "real" robot
    std::vector<double> robotSensorValues = {};
    for (DistanceSensor *sensor : DistanceSensor::filter_for_distance_sensor(this->robot->get_sensors())) {
        robotSensorValues.push_back(sensor->get_simplified_sensor_value());
    }

    // this allows to read the sensor values of all simulated particles in particles_update()
    this->updateParticleSimulation(updated_particles, false);

    // this will do the actual weights update
    std::vector<double> updated_weights = this->particles_update(robotSensorValues, this->particles_world->getRobotsList(), 0.01);

    // Particle Filter Step 3) resample: create new particles based on the weights. They will be mostly around the most previous particles with the highest weight.
    std::tuple<std::vector<std::array<double, 3>>, std::vector<double>> resampledTuple = this->particles_resample(&updated_particles,
                                                                                                                  &updated_weights,
                                                                                                                  this->N,
                                                                                                                  this->useRandomParticles,
                                                                                                                  5,
                                                                                                                  this->particles_world->get_map_bounds());

    // END of actual particle filter steps


    // calculate estimated position
    std::array<double, 3> estimatedParticle{};
    if (!this->initialLocationFinished) {
        // if in the initial phase: only the first x particles are relevant, the others are random
        int relevantParticlesCount = N - removeElementsCount;
        estimatedParticle = weightedAverageParticle(std::vector<std::array<double,3>>(updated_particles.begin(), updated_particles.begin()+relevantParticlesCount), std::vector<double>(updated_weights.begin(), updated_weights.begin()+relevantParticlesCount));
    } else {
        estimatedParticle = weightedAverageParticle(updated_particles, updated_weights);
    }


    // estimate whether the prediction is probably already accurate and if yes, add to history, if not: clear history
    // also visualize it on the map as a line
    if (!this->estimationHistory.empty()) {
        // TODO: replace this algorithm with a better one looking further in the past and calculate an uncertainty value
        double maxAcceptableMovementDistance = pow(this->robot->get_max_move_speed() / GAME_TPS * 2.5, 2); // theoretically maximum possible movement distance * X as square to prevent requirement of sqrt
        double distanceFromLastEstimation = pow(this->estimationHistory.back()[0] - estimatedParticle[0], 2) + pow(this->estimationHistory.back()[1] - estimatedParticle[1], 2);
        if (distanceFromLastEstimation >= maxAcceptableMovementDistance) {
            this->estimationHistory.clear();
            this->locationCertaintyEstimation = this->locationCertaintyEstimation > 0 ? this->locationCertaintyEstimation - 5 / GAME_TPS : 0;
            if (SHOW_WHATS_GOING_ON && !this->initialLocationFinished) this->mapLine->clearPoints();
            std::cout << "Estimated location jumped too far, this is an indication for a wrong location estimation" << std::endl;
        } else {
            this->locationCertaintyEstimation = this->locationCertaintyEstimation < 10 ? this->locationCertaintyEstimation + 1 / GAME_TPS : 10;
        }
    }
    if (SHOW_WHATS_GOING_ON) this->estimationHistory.push_back(estimatedParticle);
    this->mapLine->addPoint(cv::Point2i((int) estimatedParticle[0], (int) estimatedParticle[1]));

    // show current particles with their weights and the estimated robot location
    // nice for visualization but will impact performance!
    if (SHOW_WHATS_GOING_ON) {
        this->updateParticleSimulation(updated_particles, false, updated_weights);
        this->estimatedRobot->reposition(cv::Point2d(estimatedParticle[0], estimatedParticle[1]), estimatedParticle[2]);
        this->particles_world->show_map(true);
    }


    // save particles and weights
    this->particles = std::get<0>(resampledTuple);
    this->weights = std::get<1>(resampledTuple);


    return ParticleEvaluationData({cv::Point2d(estimatedParticle[0], estimatedParticle[1]), estimatedParticle[2], locationCertaintyEstimation >= CERTAINTY_ESTIMATION_THRESHOLD});
}


