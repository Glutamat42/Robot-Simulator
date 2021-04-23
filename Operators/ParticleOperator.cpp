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


ParticleOperator::ParticleOperator(RobotControlInterface *robot, std::string map_filename, bool benchmarkMode) : RobotOperator(robot) {
    this->BENCHMARK_MODE = benchmarkMode;
    this->secondOperator = new BasicWithNovelty(robot);
    this->particles_world = new World(map_filename, "Particle filter");
    this->particles_world->addMapObject(this->mapLine);

    if (this->BENCHMARK_MODE) {
        this->INIT_N = 5000;
        this->TARGET_N = 200;
        this->TARGET_SHOULD_BE_REACHED_AFTER_ITERS = 150;
    }

    this->N = INIT_N;

    // create initial particles distribution and show the simulation map
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

/** do the simulation part
 * - add robots (representing the particles) to the map
 * - calculate sensor values (only map boundaries, ignoring obstacles/objects)
 * - show map (if enabled)
 *
 * @param particles
 * @param showMap
 * @param weights
 */
void ParticleOperator::updateParticleSimulation(std::vector<std::array<double, 3>> particles, bool showMap, std::vector<double> weights) {
    // get sensor config to recreate (clone) the same sensors as on the robot
    std::vector<std::array<double, 2>> sensor_config;
    for (DistanceSensor *sensor : this->filter_for_distance_sensor(this->robot->get_sensors())) {
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

    // create dummy robots representing the particles
    this->particles_world->clearObjectsList(true);
    for (int i = 0; i < particles.size(); ++i) {
        std::array<double, 3> particle = particles[i];
        // TODO: creating a dummyRobot class would allow updating existing robots. This would allow to avoid recreating everything all the times
        Robot *dummyRobot = new Robot("particle",
                                      this->robot->get_radius(),
                                      this->particles_world,
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

    // show map if enabled
    if (showMap) {
        this->particles_world->show_map(true);
    }
}

/** update particle weights based on simulation
 */
std::vector<double> ParticleOperator::particles_update(std::vector<double> robotSensorValues, double lambda) {
    std::vector<Robot *> simRobots = this->particles_world->get_robots();  // cache robots here, get_robots function is very expensive
    std::vector<double> updated_weights = std::vector<double>(simRobots.size(), 1.0);
    auto d = boost::math::exponential_distribution<>{lambda};

    double sumOfWeights = 0;


    for (int i = 0; i < updated_weights.size(); ++i) {
        // collect sensor values of simulated particles
        std::vector<DistanceSensor *> sensors = this->filter_for_distance_sensor(simRobots.at(i)->get_sensors());
        std::vector<double> simulationSensorValues = {};
        for (DistanceSensor *sensor : sensors) {
            simulationSensorValues.push_back(sensor->get_simplified_sensor_value());
        }

        // calculate correlation
        for (int j = 0; j < simulationSensorValues.size(); ++j) {
            updated_weights[i] *= boost::math::pdf(d, abs(simulationSensorValues[j] - robotSensorValues[j]));
            updated_weights[i] += 1.e-300; // prevent rounding to 0
        }

        sumOfWeights += updated_weights[i];
    }

    // normalize weights: sum of weights should equal 1
    for (int i = 0; i < updated_weights.size(); ++i) {
        updated_weights[i] /= sumOfWeights;
    }

    return updated_weights;
}

std::tuple<std::vector<std::array<double, 3>>, std::vector<double>>
ParticleOperator::particles_resample(std::vector<std::array<double, 3>> *oldParticles, std::vector<double> *weights, int N, double noise) {
    // resample algorithm is explained here: https://robotics.stackexchange.com/a/481
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

    // replace random particles with random values to allow recovery in case the detection went horribly wrong
    if (this->useRandomParticles) {
        std::vector<std::array<double, 3>> randomParticles = create_uniform_particles(this->particles_world->get_map_bounds().x,
                                                                                      this->particles_world->get_map_bounds().y, N / 100);
        std::uniform_int_distribution<int> uniformIndexCreator(0, N);
        for (int i = 0; i < N / 100; ++i) {
            updatedParticles[uniformIndexCreator(generator)] = randomParticles[i];
        }
    }

    return std::tuple<std::vector<std::array<double, 3>>, std::vector<double>>(updatedParticles, updatedWeights);
}

/** loop function which will be called once every tick
 *
 */
void ParticleOperator::update() {
    // slowly decrease the amount of used particles. This allows starting with a large number and still provide good performance in the long term.
    // the most critical situation is the initial localization. Afterwards it seems to be pretty stable even with a really slow amount of particles
    if (iterationsCounter <= TARGET_SHOULD_BE_REACHED_AFTER_ITERS && iterationsCounter > 0) {
        int removeElementsCount = (INIT_N - TARGET_N) / TARGET_SHOULD_BE_REACHED_AFTER_ITERS;
        this->N -= removeElementsCount;

        // bubble sort
        // could be replaced with a more efficient algorithm if its computationally too expensive
        bool finished = false;
        for (int i = 0; !finished && i < this->particles.size(); ++i) {
            finished = true;
            // TODO: probably replacing "int j = 0" with "int j = i + 1" will improve performance and wont break anything
            for (int j = 0; j < this->particles.size(); ++j) {
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

        // drop the first particles (lowest probability)
        this->weights.erase(this->weights.begin(), this->weights.begin() + removeElementsCount);
        this->particles.erase(this->particles.begin(), this->particles.begin() + removeElementsCount);

        // if this is the last step of decaying particles also disable the random particles. If the robot still not found his location
        // then everything is already lost anyways
        if (iterationsCounter == TARGET_SHOULD_BE_REACHED_AFTER_ITERS) {
            this->useRandomParticles = false;
        }
    }
    this->iterationsCounter++;

    // using another operator for the actual movement, so this one can focus on particle filter
    this->secondOperator->update();

    // Partial implementation of steps accumulation. It might be that the algorithm works better if there is more movement between steps.
    // Especially on high TPS the movements are extremely small.
//    // accumulate movement of some steps
//    if (stepsCounter < GAME_TPS) { // once per second
//        this->stepsCounter++;
//        this->currentStepTurnedByAngle += this->robot->get_last_tick_movement_angle();
//        this->currentStepMoved += this->robot->get_last_tick_movement_distance();
//    } else {
//        this->stepsCounter = 0;
//        double angleTurned = this->robot->get this->currentStepTurnedByAngle
//    }

    // Particle Filter Step 1) particle prediction: apply estimated movement distance / angle (currently in this simulator there is no noise for those values)
    std::vector<std::array<double, 3>> updated_particles = particles_predict(&this->particles,
                                                                             this->robot->get_last_tick_movement_distance(),
                                                                             this->robot->get_last_tick_movement_angle(),
                                                                             4);

    // Particle Filter Step 2) update weights: update the weight of each particle based on how much the simulated sensor values correspond to the ones of our "real" robot
    // generate vector containing the current sensor values of the "real" robot
    std::vector<double> robotSensorValues = {};
    for (DistanceSensor *sensor : this->filter_for_distance_sensor(this->robot->get_sensors())) {
        robotSensorValues.push_back(sensor->get_simplified_sensor_value());
    }

    // this allows to read the sensor values of all simulated particles in particles_update()
    this->updateParticleSimulation(updated_particles, false);

    // this will do the actual weights update
    std::vector<double> updated_weights = this->particles_update(robotSensorValues, 0.01);



    // calculate estimated position
    std::array<double, 3> estimatedParticle = weightedAverageParticle(updated_particles, updated_weights);

    // estimate whether the prediction is probably already accurate and if yes, add to history, if not: clear history
    // also visualize it on the map as a line
    if (!this->estimationHistory.empty()) {
        // TODO: replace this algorithm with a better one looking further in the past and calculate an uncertainty value
        double maxAcceptableMovementDistance = pow(this->robot->get_max_move_speed() / GAME_TPS * 2.5, 2); // theoretically maximum possible movement distance * X as square to prevent requirement of sqrt
        double distanceFromLastEstimation = pow(this->estimationHistory.back()[0] - estimatedParticle[0], 2) + pow(this->estimationHistory.back()[1] - estimatedParticle[1], 2);
        if (distanceFromLastEstimation >= maxAcceptableMovementDistance) {
            this->estimationHistory.clear();
            this->mapLine->clearPoints();
            std::cout << "location probably not yet found" << std::endl;
        }
    }
    this->estimationHistory.push_back(estimatedParticle);
    this->mapLine->addPoint(cv::Point2i((int)estimatedParticle[0], (int)estimatedParticle[1]));

    // show current particles with their weights and the estimated robot location
    // nice for visualization but will impact performance!
    this->updateParticleSimulation(updated_particles, false, updated_weights);
    Robot *estimatedRobot = new Robot("estimation", 8, this->particles_world, cv::Point2d(estimatedParticle[0], estimatedParticle[1]), estimatedParticle[2]);
    estimatedRobot->setDrawOptions(CV_RGB(255, 255, 255));
    this->particles_world->add_object(estimatedRobot);
    this->particles_world->show_map(true);

    // Particle Filter Step 3) resample: create new particles based on the weights. They will be mostly around the most previous particles with the highest weight.
    std::tuple<std::vector<std::array<double, 3>>, std::vector<double>> resampledTuple = this->particles_resample(&updated_particles,
                                                                                                                  &updated_weights,
                                                                                                                  this->N,
                                                                                                                  5);
    // save particles and weights
    this->particles = std::get<0>(resampledTuple);
    this->weights = std::get<1>(resampledTuple);

    // exit if benchmark mode and 200 iterations passed
    if (this->BENCHMARK_MODE && iterationsCounter == 199) exit(0);
}
