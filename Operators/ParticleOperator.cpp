//
// Created by markus on 20.04.21.
//

#include <random>
#include "ParticleOperator.h"
#include "BasicWithNovelty.h"
#include <boost/math/distributions/exponential.hpp>

ParticleOperator::ParticleOperator(RobotControlInterface *robot, std::string map_filename) : RobotOperator(robot) {
    this->particles_world = new World(map_filename, "Particle filter");
    this->secondOperator = new BasicWithNovelty(robot);


    int N = 5000;     //Number of particles
    int x_range = this->particles_world->get_map_bounds().x;
    int y_range = this->particles_world->get_map_bounds().y;

    this->particles = create_uniform_particles(x_range, y_range, N);
    this->weights = std::vector<double>(N, 1.0);

    this->updateParticleSimulation(this->particles, true);
}

/** create a list of N particles with uniformly distributed positions over the map (x_range, y_range) and a normaly
 * distributed angle
 */
std::vector<std::array<double, 3>> ParticleOperator::create_uniform_particles(int x_range, int y_range, int N) {
    std::vector<std::array<double, 3>> particles;
    std::default_random_engine generator;
    std::uniform_real_distribution<double> x_distribution(0, x_range);
    std::uniform_real_distribution<double> y_distribution(0, y_range);
    std::uniform_real_distribution<double> angle_distribution(-M_PI, M_PI);

//    // TESTING
//    // exact match
//    particles.push_back(std::array<double, 3>({320,
//                                               40,
//                                               1.18*M_PI}));
//    // good match
//    particles.push_back(std::array<double, 3>({315,
//                                               45,
//                                               1.17*M_PI}));
//    // END TESTING

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
std::vector<std::array<double, 3>>
ParticleOperator::particles_predict(std::vector<std::array<double, 3>> *oldParticles,
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


void ParticleOperator::updateParticleSimulation(std::vector<std::array<double, 3>> particles, bool showMap) {
    // get sensor config to recreate (clone) the same sensors as on the robot
    std::vector<std::array<double, 2>> sensor_config;
    for (DistanceSensor *sensor : this->filter_for_distance_sensor(this->robot->get_sensors())) {
        sensor_config.push_back(std::array<double, 2>({sensor->get_sensor_angle(), sensor->get_sensor_max_distance()}));
    }

    // create dummy robots representing the particles
    this->particles_world->clearObjectsList();
    for (std::array<double, 3> particle : this->particles) {
        // TODO: creating a dummyRobot class would allow updating existing robots. This would allow to avoid recreating everything all the times
        Robot *dummyRobot = new Robot("particle",
                                      this->robot->get_radius(),
                                      cv::Point2d(particle[0], particle[1]),
                                      particle[2]);
        for (std::array<double, 2> s_config : sensor_config) {
            DistanceSensor *sensor = new DistanceSensor(this->particles_world, dummyRobot, s_config[0], s_config[1], true);
            sensor->update_sensor_data(true);
            dummyRobot->add_sensor(sensor);
        }
        this->particles_world->add_object(dummyRobot);
    }

    if (showMap) {
        this->particles_world->show_map();
    }
}


std::vector<double> ParticleOperator::particles_update(std::vector<double> robotSensorValues, double lambda) {
    std::vector<double> updated_weights = std::vector<double>(this->particles_world->get_robots().size(), 1.0);
    auto d = boost::math::exponential_distribution<>{lambda};

    double sumOfWeights = 0;

    for (int i = 0; i < updated_weights.size(); ++i) {
        // collect sensor values of simulated particles
        std::vector<DistanceSensor*> sensors = this->filter_for_distance_sensor(this->particles_world->get_robots().at(i)->get_sensors());
        std::vector<double> simulationSensorValues = {};
        for (DistanceSensor* sensor : sensors) {
            simulationSensorValues.push_back(sensor->get_simplified_sensor_value());
        }

        // calculate correlation
        for (int j = 0; j < simulationSensorValues.size(); ++j) {
            updated_weights[i] *= boost::math::pdf(d, abs(simulationSensorValues[j] - robotSensorValues[j]));
            updated_weights[i] += 1.e-300; // prevent rounding to 0
        }

        sumOfWeights += updated_weights[i];
    }

    // sum of weights should equal 1
    for (int i = 0; i < updated_weights.size(); ++i) {
        updated_weights[i] /= sumOfWeights;
    }

    return updated_weights;
}

void ParticleOperator::update() {
    this->secondOperator->update();

    std::vector<std::array<double, 3>> updated_particles = particles_predict(&this->particles,
                                                                             this->robot->get_last_tick_movement_distance(),
                                                                             this->robot->get_last_tick_movement_angle(),
                                                                             50);

    // generate vector containing the current sensor values of the "real" robot
    std::vector<double> robotSensorValues = {};
    for (DistanceSensor *sensor : this->filter_for_distance_sensor(this->robot->get_sensors())) {
        robotSensorValues.push_back(sensor->get_simplified_sensor_value());
    }

    // this allows to read the sensor values of all simulated particles in particles_update()
    this->updateParticleSimulation(updated_particles, true);

    std::vector<double> updated_weights = this->particles_update(robotSensorValues, 0.10);

    std::tuple<std::vector<std::array<double, 3>>, std::vector<double>> resampledTuple = this->particles_resample(&updated_particles, &updated_weights);
    this->particles = std::get<0>(resampledTuple);
    this->weights = std::get<1>(resampledTuple);


    std::cout << "testing ...";
}

std::tuple<std::vector<std::array<double, 3>>, std::vector<double>> ParticleOperator::particles_resample(std::vector<std::array<double, 3>> *oldParticles, std::vector<double> *weights) {
    int N = oldParticles->size();

    std::vector<double> positions = {};
    for (int i = 0; i < N; ++i) {
        positions.push_back((i + ((double) rand() / RAND_MAX)) / N);
    }

    std::vector<int> indexes(N, 0);
    std::vector<double> cumulativeSum = {};
    cumulativeSum.push_back(weights->at(0));
    for (int i=1;i<N;++i){
        cumulativeSum.push_back(cumulativeSum.at(i-1) + weights->at(i));
    }

    int i = 0;
    int j = 0;
    while (i < N && j<N) {
        if (positions[i] < cumulativeSum[j]) {
            indexes[i] = j;
            i += 1;
        }
        else {
            j += 1;
        }
    }

    std::vector<std::array<double, 3>> updatedParticles = {};
    std::vector<double> updatedWeights = {};
    double weightSum = 0;
    for (int i = 0; i < N; ++i) {
        updatedParticles.push_back( oldParticles->at(indexes[i]));
        updatedWeights.push_back(weights->at(indexes[i]));
        weightSum += weights->at(indexes[i]);
    }
    for (int i = 0; i < N; ++i) {
        weights->at(indexes[i]) /= weightSum;
    }


    return std::tuple<std::vector<std::array<double, 3>>, std::vector<double>>(updatedParticles, updatedWeights);
}
