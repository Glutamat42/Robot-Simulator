# About this project
This project contains my solutions for the exercises of the lecture Mobile Robots at the University of Applied Sciences Kempten in the summer term 2021 by Prof. Dr.-Ing. Jürgen Brauer.

This project goes beyond the scope of the exercises of this lecture and includes e.g. topics from the lecture Multimodal Sensor Systems.

Parts of this project are based on the solutions of Prof. Dr.-Ing. Jürgen Brauer. These can be found here: https://github.com/juebrauer (especially https://github.com/juebrauer/teaching_mobile_robots)
# Setup
requires libboost-dev libopencv-dev

# Project demo
In the following the self-driving scenario will be described. It's using most features this simulator currently have.

The following picture shows the simulation view. This is not what the simulated robot sees/knows. 
It represents the current state of the simulator.
The robot is represented as a red circle with a green line, representing its orientation. 
The blue lines are distance sensors measuring some noisy distance values. 
They are the only way the robot can detect his environment.
He does not have other sensors, nor he knows its location or orientation. The only other thing the robots knows is the map.

![Simulator view](./docs/assets/simulator%20view.png)

In this scenario the robot starts with random movements. 
The Operator used for this stage will only avoid crashing into walls (or unstuck if he hits a wall). 
This way the robot can collect some data required for the particle filter to locate the robot.

Over the first seconds the robot will utilize a particle filter to find his location in the known map.
Once he's certain he found his location he will calculate a path based on his estimated location to the target (in the bottom right corner of the map) with the A* algorithm.
The particle filter will continue to run, but now with way fewer particles.

![AStar](./docs/assets/astar.png)

The calculation for the path finding on this picture took ~200ms (400x400px). 
Calculating same path on the full map size (800x800px) with graphical output disabled takes around 300ms on a single core of my CPU (i7 8700k).

In the next step the robot calculates a list of waypoints based on the path found by A*.
Utilizing the estimated location and orientation the robot will start to navigate to the waypoints until he reaches the last one (the target).

![particle filter animation](./docs/assets/particle-filter.gif)


# Other features
This might or might not be a complete list of further features ;)

## Value Iteration
Currently, there is an incomplete implementation of the Reinforcement learning algorithm _Value Iteration_.
The algorithm is working, but there are still some bugs and incomplete functions (eg the path calculation is incomplete).

![Value Iteration](./docs/assets/ValueIteration.png)

On a map scaling of 2 (1/2 - 400x400 px) and without graphical output one iteration takes around 250ms. 
The code is not performance optimized, so there is plenty of room for improvement.


# Useful
## Other particle filter implementations
https://github.com/mjl/particle_filter_demo/tree/06c3c35e20df9e0d6e5d1d6eea861d51d8bba115

https://github.com/behnamasadi/Filters