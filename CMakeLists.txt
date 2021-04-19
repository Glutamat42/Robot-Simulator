cmake_minimum_required(VERSION 3.17)
project(mr_cpp_code)

set(CMAKE_CXX_STANDARD 20)

find_package(OpenCV REQUIRED PATHS "/home/markus/opencv/install")

message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

configure_file(world1.png world1.png COPYONLY)

add_executable(mr_cpp_code main.cpp interactive_objects/robot.cpp interactive_objects/robot.h world.cpp world.h interactive_objects/DistanceSensor.cpp interactive_objects/DistanceSensor.h constants.h RobotOperator.cpp RobotOperator.h interactive_objects/interfaces/RobotControlInterface.h interfaces/SensorInterface.h interactive_objects/interfaces/CollidableObject.h interactive_objects/interfaces/CollidableCircle.h helpers.h helpers.cpp interactive_objects/interfaces/CollidableCircle.cpp interactive_objects/interfaces/CollidablePoint.h interactive_objects/WallPoint.h interactive_objects/WallPoint.cpp interactive_objects/interfaces/CollidableRay.h interactive_objects/interfaces/CollidableRay.cpp interactive_objects/CollisionData.h)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} )