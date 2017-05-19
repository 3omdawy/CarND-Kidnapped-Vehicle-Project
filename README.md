# Kidnapped Vehicle Project (Localization using Particle Filters)
## Description
Your robot has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

In this project a 2 dimensional particle filter in C++ is implemented. The particle filter will be given a map and some initial localization information (analogous to what a GPS would provide). At each time step the filter will also get observation and control data. 

## Dependencies
1 - CMake, Make, GNU compiler are installed
Used installers:
	cmake-3.7.2-win64-x64.msi
	make-3.81.exe
	mingw-get-setup.exe
2 - CMake and GNU bin paths are added to "Path" environment variable

## Installation
Once you have this repository on your machine, `cd` into the repository's root directory and run the following commands from the command line:
```
mkdir build && cd build
cmake .. && make
particle_filter
```
**NOTE**
> If you encounter any problems, copy "vcvars32,bat" to build directory and run the command `vcvars32` to set environment variables
> If make command does not work try: `cmake .. -G "Unix Makefiles" && make`

If everything worked you should see something like the following output:

Time step: 2443
Cumulative mean weighted error: x 0.112 y 0.106 yaw 0.003
Runtime (sec): 32.08
Success! Your particle filter passed!

## Common Usage
Particle filters are powerful in localization techniques in realtime. Localization is essential for self-driving cars.

# Repository Files
The directory structure of this repository is as follows:

```
root
|   vcvars32,bat
|   CMakeLists.txt
|   README.md
|
|___data
|   |   control_data.txt
|   |   gt_data.txt
|   |   map_data.txt
|   |
|   |___observation
|       |   observations_000001.txt
|       |   ... 
|       |   observations_002444.txt
|   
|___src
    |   helper_functions.h
    |   main.cpp
    |   map.h
    |   particle_filter.cpp
    |   particle_filter.h
```

The file containing the implementation of the filter is `particle_filter.cpp` in the `src` directory.
If you are interested, take a look at `src/main.cpp` as well. This file contains the code that will actually be running the particle filter and calling the associated methods.

## Inputs to the Particle Filter
You can find the inputs to the particle filter in the `data` directory. 

#### The Map*
`map_data.txt` includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns
1. x position
2. y position
3. landmark id

> * Map data provided by 3D Mapping Solutions GmbH.


#### Control Data
`control_data.txt` contains rows of control data. Each row corresponds to the control data for the corresponding time step. The two columns represent
1. vehicle speed (in meters per second)
2. vehicle yaw rate (in radians per second)

#### Observation Data
The `observation` directory includes around 2000 files. Each file is numbered according to the timestep in which that observation takes place. 

These files contain observation data for all "observable" landmarks. Here observable means the landmark is sufficiently close to the vehicle. Each row in these files corresponds to a single landmark. The two columns represent:
1. x distance to the landmark in meters (right is positive) RELATIVE TO THE VEHICLE. 
2. y distance to the landmark in meters (forward is positive) RELATIVE TO THE VEHICLE.

> **NOTE**
> The vehicle's coordinate system is NOT the map coordinate system.