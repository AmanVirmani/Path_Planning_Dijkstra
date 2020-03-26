# Path_Planning_Dijkstra
[![Build Status](https://travis-ci.org/AmanVirmani/Path_Planning_Dijkstra.svg?branch=master)](https://travis-ci.org/AmanVirmani/Path_Planning_Dijkstra)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

## Overview
Implementation of Dijkstra algorithm for a Point and Rigid Robot

## Build Instructions

Run the following bash command to do path planning for a point robot
```bash
python3 Dijkstra_point.py -s 0 0 -g 50 30
```

For a rigid robot, enter the following command.
```bash
python3 Dijkstra_rigid.py -r <radius> -c <clearance> -s <x y| start node> -g <x y| goal node> 
```

## Output

Two gifs are generated as output. 
1. explored.gif : shows the explored path in green.

2. output.gif : shows the optimal path selected after exploration

## Dependencies

The following dependencies must be installed.

1. python3.5 or above 
2. numpy 
3. opencv 3.4 or above
4. imagio 

Enter the given commands in bash terminal to install the dependencies.
```bash
sudo apt-get install python3
pip3 install numpy opencv-python imageio
```

