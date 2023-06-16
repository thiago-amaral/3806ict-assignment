# 3806ICT Assignment 3 by:

-  Callam Hartley | s5113156
-  Daniel Jacobsen | s5262721
-  Todd Cooper | s2681289
-  James Hudson | s5182091

Clone the repository into a catkin workspace folder (under catkin_ws/src)
Compile the binaries in catkin_ws/src using catkin_make
Launch the world:
roslaunch assignment_3 launch_world.launch

Spin up services hosted by the updateGrid node:
rosrun assignment_3 updateGrid

Main file is search_rescue:
rosrun assignment_3 search_rescue

in search_rescue.cpp, PAT_EXE_DIR is defined as the direct path to the PAT executable.
If this is in a different directory on your machine, either relocate PAT to the expected
path ("/Desktop/MONO-PAT-v3.6.0/PAT3.Console.exe") or change this variable to the correct
path.
