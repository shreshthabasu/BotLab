
## Reposiory Structure

### Directories

1. bin/
    - where all built binaries are located
    - you'll be using this directory a lot
    
2. data/
    - where data needed to run parts of the assignment are located
    - log files and target files for SLAM and exploration are here
    
3. lcmtypes/
    - where the .lcm type definitions are located
    - the generated types are stored in src/lcmtypes
    
4. lib/
    - where static libraries are saved during the build process
    - you should never need to manually do anything in this directory
    
5. src/
    - where all source code for botlab is located
    - the subdirectories will have a further description of their contents
    

### Files 

setenv.sh
    - a script for setting environment variables needed for running Vx applications
    - run this script before running botgui in a terminal on your laptop
    - run via `. setenv.sh` -- note the space
    
___________________________________________________________________________________________
## Run the code:

- ./bin/botgui : GUI for robot in the environment
- run a log file form /data using lcm-logplayer-gui
- Other files in the /bin directory based on which part of the code you want to run
- /src/sim/sim.py path_to_map(present in /data)

### Occupancy grid mapping
Implemented occupancy grid mapping using sensor inversion model.
- run a log, botgui
- run ./slam --mapping-only

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

    
 
