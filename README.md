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

- **GUI**: ```./bin/botgui``
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
<img src="https://github.com/shreshthabasu/BotLab/blob/master/media/occupancygridmapping.gif", width="480">
</p>
*Occupancy Grid Mapping*

### SLAM
- run botgui
- run ./slam

### SLAM with path planning and exploration
- run sim.py, botgui
- in /bin, run:
    - ./timesync
    - ./motion_controller
    - ./slam
    - ./exploration

    
 
