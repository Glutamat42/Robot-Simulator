//
// Created by markus on 23.04.21.
//

#ifndef MR_CPP_CODE_SELFDRIVINGOPERATOR_H
#define MR_CPP_CODE_SELFDRIVINGOPERATOR_H


#include "../lib/ParticleFilter.h"
#include "RobotOperator.h"
#include "../lib/AStar/FastAStar.h"
#include <boost/circular_buffer.hpp>

class SelfDrivingOperator : public RobotOperator {
private:
    RobotOperator *secondOperator;
    ParticleFilter particleFilter;
    FastAStar aStar;
    std::vector<cv::Point2d> path;
    long waypointId;
    boost::circular_buffer<cv::Point2d> positionHistory;

    long getNextWaypointIndex(long currentWaypointIndex);

    /**
     *
     * @param currentLocation
     * @param currentAngle
     * @return true if target reached
     */
    bool navigate(cv::Point2d currentLocation, double currentAngle);

    /** This will move to the currently selected waypointId. There will no checks happen eg whether the robot should select the next waypoint.
     * Should not be called directly (call navigate(...) instead)
     *
     * @param AngleOfCurrentWaypoint if the angle (in relation to vector x:1, y0) is already known pass it here, it will save some time
     */
    void moveToWaypoint(cv::Point2d currentLocation, double currentAngle);
public:
    SelfDrivingOperator(RobotControlInterface *robot, std::string map_filename, bool benchmarkMode = false);

    void update() override;
};


#endif //MR_CPP_CODE_SELFDRIVINGOPERATOR_H
