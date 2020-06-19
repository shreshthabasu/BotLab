# BOTLAB
This lab involves writing code to control an mBot which moves with differential drive and is equipped with magnetic wheel encoders, a 2D Lidar, and a MEMS 3-axis IMU. Log files are created by recording data from these sensors while the mBot was driven. All implementations use the collected data and maps.
The goal of this lab is to:
* Build an occupancy grid, action model  and sensor model of the virtual mBot
* Implement path planning and navigation algorithms for the virtual mBot

## Repository Structure

### Directories

1. bin/
    - where all built binaries are located
    
2. data/
    - log files and maps are located
    
3. lcmtypes/
    - where the .lcm type definitions are located
    - the generated types are stored in src/lcmtypes
    
4. lib/
    - where static libraries are saved during the build process
    
5. src/
    - where all source code for botlab is located

### Files 

setenv.sh
    - a script for setting environment variables needed for running Vx applications
    - run this script before running botgui in a terminal on your laptop
    - run via `. setenv.sh`
    
___________________________________________________________________________________________
## Run the code:

- **GUI**: ```./bin/botgui```
- **Logfile**: run a log file in /data directory: ```lcm-logplayer-gui name_of_logfile.log```
- **Binaries**: Other files in the ```/bin directory``` based on which part of the code you want to run
- **Simulator**: Run the bot simulator that mimics actual robot: ```/src/sim/sim.py path_to_map(present in /data)```

### Occupancy grid mapping
Implemented occupancy grid mapping which involves assigning log odds for every cell in the map that
each lidar ray can pass through or touch. To account for the robot moving while scanning, pose interpolation is
implemented to record the robot pose for every scan reading. All the cells in the map is initialized with 0.
For each ray in the laser scan:
* Log odds of the end cell is incremented by 3 indicating that it is occupied
* Brasenhams algorithm is used to find the list of cells between the robot's start position and the end cell. Log odds for each of these cells were decremented by 1 indicating that it's empty.
Main code for this section is found in ```src/slam/mapping.cpp```

**Implementation:**
- run a log, botgui
- run ./slam --mapping-only
<p align="left">
<img src="https://github.com/shreshthabasu/BotLab/blob/master/media/occupancygridmapping.gif", width="400">
</p>

### SLAM
As the name suggests, SLAM refers to simulatenous localization and mapping. The mapping was alredy covered in the previous section. In order to localize the robot accurately, we use an odometry based action model and a simplified likelihood Field Model for sensor model.
* **Action Model**: Odometry data calculated using information from the wheel encoders is used to calculate the robot position at time t given the robot position and control input at time t-1. The errors associated with control inputs are modelled as gaussian noise. Code for this section is found in ```src/slam/action_model.cpp```
* **Sensor Model**: Since odometry data is unreliable over time, we make use of a sensor model whose purpose is to find the probability of the lidar scan matching the hypothesis particle pose in a map. Code for this section is found in ```src/slam/sensor_model.cpp```
* **Particle Filter**: Particles(200) are initialized at the robot start pose. This distribution represents the prior. Proposal distribution is calculated by applying the action model on the prior. Next, posterior is calculated by applying the sensor model on the proposal. The robot position is estimated by calculating the average position of the posterior. Finally, the prior is calcuated for the next iteration using the low variance resampling algorithm. Thus, the process continues. Code for this section is found in ```src/slam/particle_filter.cpp```

**Implementation**
- run botgui
- run ./slam
<p align="left">
<img src="https://github.com/shreshthabasu/BotLab/blob/master/media/slam.gif", width="400">
</p>
The path taken by the yellow robot represents the slam trajectory while that taken by the red robot represents the trajectory based on odometry pose.

### SLAM with path planning and exploration
* **Path Planning**: In order to plan a path from the current cell to the goal cell, 8 way A* path planning was implemented. Obstacle distance grid mapping was implemented wherein each cell stored a value of its distance from the closest obstacle to it. This value was taken into consideration while calculating the fCost of each node in A*. Cells closer to the obstacle were penalized. The code for this section can be found in ```src/planning/astar.cpp``` and ```src/planning/obstacle_distance_grid.cpp```.
* **Exploration**: The robot explores a map until frontiers(cells with 0 probability) are present. The robot arbitrarily picks a frontier and chooses a cell close to it as the goal. It uses A* path planning to travel there. Once the goal is reached, the robot plans a path to another frontier using the same logic. Once there are no more frontiers, the robot travels to its home pose. The code for this section can be found in ```src/planning/frontiers.cpp``` and ```src/planning/exploration.cpp```.

**Implementation**
- run sim.py, botgui
- in /bin, run:
    - ./timesync
    - ./motion_controller
    - ./slam
    - ./exploration

 <p align="left">
<img src="https://github.com/shreshthabasu/BotLab/blob/master/media/exploration.gif", width="400">
</p>
 
