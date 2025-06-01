# 3806ICT Assignment by:

-  Pitiputt Hanson
-  Thiago Guerino Amaral
-  Luke Edwards
-  Satyam Sharma

This project requires an Ubuntu-based setup with ROS, Gazebo, PAT, and Ollama installed. It simulates an autonomous underwater vehicle (AUV) navigating a discretised 2D grid environment. Each grid cell may contain a survivor, a hostile entity, or a marker indicating whether it has been visited.

Gazebo provides real-time simulation of the environment, while ROS handles the AUV’s main control loop. The AUV uses Ollama to generate plans for high-level goals—such as "return home" and invokes PAT from the command line for path finding. PAT outputs a sequence of actions to a text file, which ROS then reads and executes step-by-step. The robot’s movement is guided by these plans, using simulated sensors to adapt to its surroundings during execution.

To successfully run this project on your own machine:

Ensure Ollama is installed, llama2 is pulled down and Ollama's python library is installed. If not, run the following setup commands:
1. `sudo apt install ollama python3-pip`
2. `ollama pull llama2`
3. `pip install ollama`

Then, to run the simulation:
1. Clone this repository into the catkin workspace source folder (`catkin_ws/src`).
2. Compile the code using catkin_make.
3. Launch the world: `roslaunch assignment_3 launch_world.launch`
4. Run the update_grid node: `rosrun assignment_3 update_grid` in a new terminal window.
5. Run the planner_node: `rosrun assignment_3 planner_node.py` in a new terminal window.
5. Run the search_rescue node: `rosrun assignment_3 search_rescue` in a new terminal window.

**Note: This project relies on a hardcoded path to the PAT installation. By default, it is set to `/Desktop/MONO-PAT-v3.6.0/PAT3.Console.exe`. If PAT is not located at this path, you will need to either move PAT to that location or update the PAT_EXE_DIR constant in the code. This `#define` can be found in `search_rescue.cpp` at line 29.**
