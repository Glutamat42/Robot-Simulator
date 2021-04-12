cmake_minimum_required(VERSION 3.17)
project(mr_cpp_code)

set(CMAKE_CXX_STANDARD 20)

find_package(OpenCV REQUIRED PATHS "/home/markus/opencv/install")

message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

configure_file(world1.png world1.png COPYONLY)

add_executable(mr_cpp_code main.cpp robot.cpp robot.h world.cpp world.h sensor.cpp sensor.h constants.h RobotOperator.cpp RobotOperator.h interfaces/RobotControlInterface.h)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} )