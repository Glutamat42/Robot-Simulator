//
// Created by markus on 20.04.21.
//

#include <random>
#include "ParticleOperator.h"
#include "BasicWithNovelty.h"

ParticleOperator::ParticleOperator(RobotControlInterface *robot, std::string map_filename) : RobotOperator(robot) {
    this->particles_world = new World(map_filename, "Particle filter");
    this->secondOperator = new BasicWithNovelty(robot);


    int N = 50;     //Number of particles
    int x_range = this->particles_world->get_map_bounds().x;
    int y_range = this->particles_world->get_map_bounds().y;

    this->particles = create_uniform_particles(x_range, y_range, N);
    this->weights = std::vector<double>(N, 1.0);

    this->updateParticleSimulation(true);
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

    for (int i = 0; i < N; i++) {
        particles.push_back(std::array<double, 3>({x_distribution(generator),
                                                   y_distribution(generator),
                                                   angle_distribution(generator)}));
    }
    return particles;
}

/** do prediction inplace
 * update particle position & angle based on the (estimated) moved distance and angle
 */
std::vector<std::array<double, 3>>
ParticleOperator::predict(std::vector<std::array<double, 3>> *particles,
                          double move_distance,
                          double move_angle,
                          double standard_deviation) {
    std::vector<std::array<double, 3>> updated_particles;

    // init normal_distribution
    std::random_device rd{};
    std::mt19937 gen{rd()};
    std::normal_distribution<> d{0, standard_deviation};

    // update particles
    for (std::array<double, 3> particle : *particles) {
        // update moved distance and angle with a norm distribution value (x * ((100 + <norm_dist>) / 100))
        double distance_with_distribution = move_distance * ((100 + d(gen)) / 100);
        double angle_with_distribution = move_angle * ((100 + d(gen)) / 100);

        // update particles with distance and angle
        updated_particles.push_back(std::array<double, 3>(
                {
                        particle[0] + cos(angle_with_distribution) * distance_with_distribution,
                        particle[1] + sin(angle_with_distribution) * distance_with_distribution,
                        particle[2] + angle_with_distribution
                }));
    }

    return updated_particles;
}


void setUp() {

}

void ParticleOperator::updateParticleSimulation(bool showMap) {
    // get sensor config to recreate the same sensors as on the robot
    std::vector<std::array<double, 2>> sensor_config;
    for (DistanceSensor* sensor : this->filter_for_distance_sensor(this->robot->get_sensors())) {
        sensor_config.push_back(std::array<double, 2>({sensor->get_sensor_angle(), sensor->get_sensor_max_distance()}));
    }

    // create dummy robots representing the particles
    this->particles_world->clearObjectsList();
    for (std::array<double, 3> particle : this->particles) {
        // TODO: creating a dummyRobot class would allow updating existing robots. This would allow to avoid recreating everything all the times
        Robot* dummyRobot = new Robot("particle",
                                      this->robot->get_radius(),
                                      cv::Point2d(particle[0],particle[1]),
                                      particle[2]);
        for (std::array<double, 2> s_config : sensor_config) {
            DistanceSensor* sensor = new DistanceSensor(this->particles_world, dummyRobot, s_config[0], s_config[1]);
            sensor->update_sensor_data(true);
            dummyRobot->add_sensor(sensor);
        }
        this->particles_world->add_object(dummyRobot);
    }

    if(showMap) {
        this->particles_world->show_map();
    }
}


void ParticleOperator::update() {
    this->secondOperator->update();

    std::vector<std::array<double, 3>> updated_particles = predict(&particles,
                                                                   this->robot->get_last_tick_movement_distance(),
                                                                   this->robot->get_last_tick_movement_angle());
    this->updateParticleSimulation(true);

//    std::cout << "testing ...";
}
