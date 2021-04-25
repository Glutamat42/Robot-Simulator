cmake_minimum_required(VERSION 3.17)
project(mr_cpp_code)

set(CMAKE_CXX_STANDARD 20)

find_package(OpenCV REQUIRED PATHS "/home/markus/opencv/install")

message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

configure_file(assets/maps/world1.png assets/maps/world1.png COPYONLY)

add_executable(mr_cpp_code main.cpp Simulator/interactive_objects/robot.cpp Simulator/interactive_objects/robot.h Simulator/world/world.cpp Simulator/world/world.h Simulator/interactive_objects/DistanceSensor.cpp Simulator/interactive_objects/DistanceSensor.h Simulator/constants.h Operators/RobotOperator.cpp Operators/RobotOperator.h Simulator/interactive_objects/interfaces/RobotControlInterface.h Simulator/interfaces/SensorInterface.h Simulator/interactive_objects/interfaces/CollidableObject.h Simulator/interactive_objects/interfaces/CollidableCircle.h lib/helpers.h lib/helpers.cpp Simulator/interactive_objects/interfaces/CollidableCircle.cpp Simulator/interactive_objects/interfaces/CollidablePoint.h Simulator/interactive_objects/WallPoint.h Simulator/interactive_objects/WallPoint.cpp Simulator/interactive_objects/interfaces/CollidableRay.h Simulator/interactive_objects/interfaces/CollidableRay.cpp Simulator/interactive_objects/CollisionData.h Scenario/SimulationScenario.cpp Scenario/SimulationScenario.h Scenario/RobotArmy.cpp Scenario/RobotArmy.h Scenario/TestRobots.cpp Scenario/TestRobots.h Operators/BasicWithNovelty.cpp Operators/BasicWithNovelty.h Operators/ParticleOperator.cpp Operators/ParticleOperator.h Scenario/ParticleScenario.cpp Scenario/ParticleScenario.h lib/FastMap.h Simulator/world/MapObject.h Simulator/world/MapLine.h Simulator/world/MapLine.cpp Scenario/AStarScenario.cpp Scenario/AStarScenario.h Operators/AStarOperator.cpp Operators/AStarOperator.h lib/ParticleFilter.cpp lib/ParticleFilter.h Operators/SelfDrivingOperator.cpp Operators/SelfDrivingOperator.h Scenario/SelfDrivingScenario.cpp Scenario/SelfDrivingScenario.h lib/DummyRobot.cpp lib/DummyRobot.h Simulator/world/MapRobot.cpp Simulator/world/MapRobot.h lib/AStar/FastAStar.cpp lib/AStar/FastAStar.h lib/AStar/AStarDataMap.h lib/AStar/AStarDataStructures.h)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} )