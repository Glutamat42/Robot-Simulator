//
// Created by markus on 23.04.21.
//

#ifndef MR_CPP_CODE_PARTICLEFILTER_H
#define MR_CPP_CODE_PARTICLEFILTER_H


#include <string>
#include "../Simulator/interactive_objects/interfaces/RobotControlInterface.h"
#include "../Simulator/world/MapLine.h"

struct ParticleEvaluationData {
    cv::Point2d currentLocation;
    double angle;
    bool estimationSeemsValid;
};

class ParticleFilter {
private:
    // variables
    RobotControlInterface *robot;
    World *particles_world;
    MapLine* mapLine = new MapLine();

    int N; //Number of particles
    std::vector<std::array<double, 3>> particles;
    std::vector<double> weights;
    std::vector<std::array<double, 3>> estimationHistory;

    int iterationsCounter = 0;
    bool useRandomParticles = true;

    // The following values should be seen as constants for this operator. They might be changed in constructor for some special cases like benchmarking
    bool BENCHMARK_MODE = false;
    int INIT_N = 5000;  // The following variables are required for the decaying particles functionality
    int TARGET_N = 75;
    int TARGET_SHOULD_BE_REACHED_AFTER_ITERS = 200;

    // -- functions --

    /** create a list of N particles with uniformly distributed positions over the grayscaleMap (x_range, y_range) and a normaly
     * distributed angle
     */
    std::vector<std::array<double, 3>> static create_uniform_particles(double x_range, double y_range, int N);

    /** do the simulation part
     * - add robots (representing the particles) to the map
     * - calculate sensor values (only map boundaries, ignoring obstacles/objects)
     * - show map (if enabled)
     *
     * @param particles
     * @param showMap
     * @param weights
     */
    void updateParticleSimulation(std::vector<std::array<double, 3>> particles, bool showMap, std::vector<double> weights = {});

    /** do prediction
     * update particle position & angle based on the (estimated) moved distance and angle
     */
    std::vector<std::array<double, 3>> static
    particles_predict(std::vector<std::array<double, 3>> *oldParticles, double move_distance, double move_angle, double standard_deviation = 1);

    /** update particle weights based on simulation */
    std::vector<double> static particles_update(std::vector<double> robotSensorValues, std::vector<Robot *> simRobots, double lambda = 0.5);

    /** redistribute the particles
     *
     * @param oldParticles
     * @param weights
     * @param N
     * @param enableRandomParticles replace some randomly selected particles with new random particles
     * @param noise
     * @param mapBounds required for random particles (enableRandomParticles)
     * @return
     */
    std::tuple<std::vector<std::array<double, 3>>, std::vector<double>> static
    particles_resample(std::vector<std::array<double, 3>> *oldParticles, std::vector<double> *weights, int N, bool enableRandomParticles = false, double noise = 10, cv::Point2d mapBounds = cv::Point2d());
public:
    ParticleFilter(RobotControlInterface *robot, std::string map_filename, bool benchmarkMode);

    /** calling this function will do one update step of the prediction
     *
     * @param distance moved distance, if not provided it will use the last ticks movement of the robot
     * @param angle turned angle, if not provided it will use the last ticks tunging of the robot
     * @return
     */
    ParticleEvaluationData update(double distance, double angle);

    /** wrapper for update(double distance = -1, double angle), will get distance and angle from the robots last movement */
    ParticleEvaluationData update();
};


#endif //MR_CPP_CODE_PARTICLEFILTER_H
