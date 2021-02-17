Goal of this project is to implement a 2D particle filter to localize a robot, given map of the location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

Main source code for the particle filter that I implemented can be found in src/particle_filter.cpp src/particle_filter.hpp, remaining files contain pre-implemented code to connect to a web sockets for communication with the silumator, reading in map data etc.,

Tools Required:
  1) CMake >= 3.5
  2) gcc, g++ 5.4.0

Download & Build Instructions:
1) Download & unzip the content to 'ParticleFilter-Localization' directory
  * Contents
    |
    +--data--+
    |        |
    |        +--map_data.txt
    |
    |
    +--src-- (Contain source code for the 2D particle filter project)
    |
    +--third_party_src--(Contain source code for third_party libraries & script to download simulator)
    |
    +--CMakeLists.txt
    |
    +--README.txt

2) Build third party libraries & dependencies
  * Required thirdparty libraries source code is available in 'third_party_src' directory
  * cd third_party_src/ && chmod u+x install_third_party.sh && ./install_third_party.sh
  * if the third_party install is successful you should see the a 'third_party' directory(with 'uWebSockets' sub-directory in it) & 'term2_sim_linux' directory in the root of the project

3) Build the particle filter application
  * From root directory, mkdir -p build && cd build && cmake .. && make
  * After successful building, particle filter application binary can be found in build directory

Running Application:
1) cd build && ./particle_filter
  This will start the particle filter & waits for simulation input
2) on a new terminal, start the silumator
  From root directory, cd term2_sim_linux && chmod u+x term2_sim.x86_64 && ./term2_sim.x86_64 &
3) Select "Project 3: Kidnapped Vehicle" by pressing "next" in simulator window
4) Press "start" to start the simulation & you can see the blue circle(localized location of the car based on Map, noisy sensor & control data) following the car


INPUT: values provided by the simulator & external files to the c++ program
// sense noisy position data from the simulator
  ["sense_x"] 
  ["sense_y"] 
  ["sense_theta"] 

// get the previous velocity and yaw rate to predict the particle's transitioned state
  ["previous_velocity"]
  ["previous_yawrate"]

// receive noisy observation data from the simulator, in a respective list of x/y values
  ["sense_observations_x"] 
  ["sense_observations_y"] 

  data/map_data.txt includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns
    1. x position
    2. y position
    3. landmark id

OUTPUT: values provided by the c++ program to the simulator

// best particle values used for calculating the error evaluation
  ["best_particle_x"]
  ["best_particle_y"]
  ["best_particle_theta"] 

//Optional message data used for debugging particle's sensing and associations
// for respective (x,y) sensed positions ID label 
  ["best_particle_associations"]

// for respective (x,y) sensed positions
  ["best_particle_sense_x"] <= list of sensed x positions
  ["best_particle_sense_y"] <= list of sensed y positions
