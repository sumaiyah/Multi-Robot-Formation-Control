# Multi-Robot-Formation-Control

A **group project** implemented in February 2020 for the third year Mobile Robot Systems course at the University of Cambridge

*Project contributors: Benjamin Philps (BenjaminPhi5) and Ajay Ahir (DoodleBobBuffPants)*

The results of the project can be found in a video I compiled (`MultiRobotFormationControl.mp4`)

An overview of the project work can be found in the presentation `MultiRobotFormationControl.pptx`.

## Requirements
- Copy the 'turtlebot3', 'turtlebot3_msgs', and 'turtlebot3_simulations' folders from 'catkin_ws/src/' from the original exercises
- Copy the 'project' folder into 'catkin_ws/src/exercises' and run the code from there.

## To Run
- Launch [Gazebo](http://gazebosim.org) using the environment you wish
- Go to init_formations.py and set MAP_PARAMS to the map you want (e.g SIMPLE_MAP)
- If you want to run RRT, set RUN_RRT to True
- Predefined paths are stored in the precomputed_rrt_paths.py file
*NOTE: If you change the starting positions of the robots in the environment, you need to recompute a new path, then either store it in the precomputed paths file or reset the starting positions*

To run the decentralised version, start any map as usual then launch formation_move_decentralized.launch for each robot, supplying id as an arg (e.g id:=1)
