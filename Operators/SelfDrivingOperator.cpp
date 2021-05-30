//
// Created by markus on 23.04.21.
//

#include "SelfDrivingOperator.h"
#include "BasicWithNovelty.h"
#include "../lib/helpers.h"

const double EXTRA_PADDING = 8;
const int WAYPOINT_DISTANCE = 10;
const double ACCEPTABLE_DISTANCE_MULTIPLIER = 3;
const double UNSTUCK_HISTORY_DURATION_IN_SECONDS = 5;
const double UNSTUCK_STUCK_IF_MOVED_LESS_THAN = 8;

SelfDrivingOperator::SelfDrivingOperator(RobotControlInterface *robot,
                                         std::string map_filename,
                                         bool benchmarkMode) : RobotOperator(robot),
                                                               particleFilter(robot, map_filename),
                                                               aStar(map_filename, robot->get_radius() + EXTRA_PADDING, 2
                                                               ) {
    if (benchmarkMode) std::cout << "Benchmark mode support has been removed from the particle filter class. If required implement it on Scenario/Operator level" << std::endl;
    this->secondOperator = new BasicWithNovelty(robot);
    this->positionHistory = boost::circular_buffer<cv::Point2d>(UNSTUCK_HISTORY_DURATION_IN_SECONDS * GAME_TPS);
}


void SelfDrivingOperator::update() {
    cv::Point2d targetPosition(725, 725);


    // first find/validate the robots location
    ParticleEvaluationData particleFilterEstimation = this->particleFilter.update();
    if (!particleFilterEstimation.estimationSeemsValid) {
        this->secondOperator->update();
        return;
    }

    // check if robot is stuck
    cv::Point2d relativeDistanceToOldPosition = particleFilterEstimation.currentLocation - this->positionHistory.front();
    double squaredMovementDistance = relativeDistanceToOldPosition.x * relativeDistanceToOldPosition.x + relativeDistanceToOldPosition.y * relativeDistanceToOldPosition.y;
    if (this->positionHistory.full() && squaredMovementDistance < (UNSTUCK_STUCK_IF_MOVED_LESS_THAN * UNSTUCK_STUCK_IF_MOVED_LESS_THAN)) {
        std::cout << "STUCK" << std::endl;
    }
    this->positionHistory.push_back(particleFilterEstimation.currentLocation);


    // If there is already a path
    if (!this->path.empty()) {
        // check distance to next waypoint
        cv::Point2d relativeDistance = this->path[this->waypointId] - particleFilterEstimation.currentLocation;
        long squaredDistanceToWaypoint = relativeDistance.x * relativeDistance.x + relativeDistance.y * relativeDistance.y;

        if (squaredDistanceToWaypoint > ((WAYPOINT_DISTANCE + this->robot->get_radius()) * (WAYPOINT_DISTANCE + this->robot->get_radius()) * ACCEPTABLE_DISTANCE_MULTIPLIER)) {
            std::cout << "Too far away from next waypoint -> drop current route and recalculate it" << std::endl;
            this->path.clear();
            this->waypointId = 0;
            // have to run A* again. Skipping the navigation (in else block) and continue with path finding
        } else {
            // we have a path and everything looks fine, so we can start to navigate to the target
            bool targetReached = navigate(particleFilterEstimation.currentLocation, particleFilterEstimation.angle);
            if (targetReached) {
                std::cout << "reached target" << std::endl;
                cv::waitKey(0);
            } else {
                return; // everything done for this step
            }
        }
    }

    // use A* to find a path
    if (this->path.empty()) {
        if(this->aStar.setAStarParameters(particleFilterEstimation.currentLocation, targetPosition)) {
            this->path = this->aStar.run();
            this->waypointId = 0;
        }
    }
}

long SelfDrivingOperator::getNextWaypointIndex(long currentWaypointIndex) {
    long newIndex = currentWaypointIndex + 1 * WAYPOINT_DISTANCE / this->aStar.getMapScaling();
    if (newIndex >= this->path.size()) newIndex = (long) this->path.size() - 1;
    return newIndex;
}

bool SelfDrivingOperator::navigate(cv::Point2d currentLocation, double currentAngle) {
    // calculate distance again, could get this from the outside to optimize performance
    cv::Point2d relativeDistanceFromCurrentLocation = this->path[this->waypointId] - currentLocation;
    long squaredDistanceToWaypoint = relativeDistanceFromCurrentLocation.x * relativeDistanceFromCurrentLocation.x + relativeDistanceFromCurrentLocation.y * relativeDistanceFromCurrentLocation.y;

    // check if robot is already over the waypoint
    // could cache squared robot radius to improve performance
    if (squaredDistanceToWaypoint < this->robot->get_radius() * this->robot->get_radius()) {
        if (this->waypointId == this->path.size() - 1) {
            return true; // last waypoint reached
        } else {
            // on waypoint, but not last one -> move to next one
            this->waypointId = this->getNextWaypointIndex(this->waypointId);
        }
    }

    moveToWaypoint(currentLocation, currentAngle);
    return false;
}

void SelfDrivingOperator::moveToWaypoint(cv::Point2d currentLocation, double currentAngle) {
    // recalculate those things because its possible that the waypoint id changed since before. This has definitely also optimization potential
    cv::Point2d currentWaypoint = this->path[this->waypointId];
    cv::Point2d relativeDistanceToNextWaypoint = currentWaypoint - currentLocation;
    double distanceToWaypoint = sqrt(relativeDistanceToNextWaypoint.x * relativeDistanceToNextWaypoint.x + relativeDistanceToNextWaypoint.y * relativeDistanceToNextWaypoint.y);
    double angleToWaypoint = atan2(relativeDistanceToNextWaypoint.y, relativeDistanceToNextWaypoint.x) - currentAngle; // potential problems if robot is around -180°

//    this->robot->set_turn_speed(this->robot->get_max_turn_rate() * angleToWaypoint);
    this->robot->set_turn_speed(this->robot->get_max_turn_rate() * sgn(angleToWaypoint));
    this->robot->set_target_turn_angle(angleToWaypoint);

    double speedMultiplier = abs(angleToWaypoint) < 1 ? 1 - abs(angleToWaypoint) : 0;
    this->robot->set_speed(this->robot->get_max_move_speed() * speedMultiplier);
    this->robot->set_target_move_distance(distanceToWaypoint);
}
