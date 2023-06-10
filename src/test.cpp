#include <ros/ros.h>
#include "assignment_3/UpdateGrid.h"
#include "assignment_3/Sensors.h"
#include "communal_defines.cpp"
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <vector>
#include <random>
#include <fstream>
#include <queue>
#include <string>
#include <utility>

// ros::ServiceClient sensorClient;
std::string homeDir = getenv("HOME");

#define PAT_EXE_DIR homeDir + "/Desktop/MONO-PAT-v3.6.0/PAT3.Console.exe"
#define PAT_PATH_CSP_DIR homeDir + "/catkin_ws/src/3806ict_assignment_3/pat/path.csp"
#define PAT_OUTPUT_DIR homeDir + "/catkin_ws/src/3806ict_assignment_3/pat/output.txt"
std::string PAT_CMD = "mono " + PAT_EXE_DIR + " -csp " + PAT_PATH_CSP_DIR + " " + PAT_OUTPUT_DIR;

void update_world(assignment_3::Sensors &srv, int (&curr_world)[BOARD_H][BOARD_W])
{
	curr_world[srv.request.newSubXIndex][srv.request.newSubYIndex] = VISITED;
	// east is col + 1
	if (srv.response.bombEast)
		for (int i = 0; i < srv.request.sensorRange; i++)
			if (srv.response.eastRadar[i])
			{
				curr_world[srv.request.newSubXIndex][srv.request.newSubYIndex + 1 + i] = HOSTILE;
				ROS_INFO("Bomb detected east!");
			}
	// west is col - 1
	if (srv.response.bombWest)
		for (int i = 0; i < srv.request.sensorRange; i++)
			if (srv.response.westRadar[i])
			{
				curr_world[srv.request.newSubXIndex][srv.request.newSubYIndex - 1 - i] = HOSTILE;
				ROS_INFO("Bomb detected west!");
			}
	// north is row - 1
	if (srv.response.bombNorth)
		for (int i = 0; i < srv.request.sensorRange; i++)
			if (srv.response.northRadar[i])
			{
				curr_world[srv.request.newSubXIndex - 1 - i][srv.request.newSubYIndex] = HOSTILE;
				ROS_INFO("Bomb detected north!");
			}
	// south is row + 1
	if (srv.response.bombSouth)
		for (int i = 0; i < srv.request.sensorRange; i++)
			if (srv.response.southRadar[i])
			{
				curr_world[srv.request.newSubXIndex + 1 + i][srv.request.newSubYIndex] = HOSTILE;
				ROS_INFO("Bomb detected south!");
			}
}

void update_true_world(int old_x, int old_y, std::pair<int, int> new_coords, int (&true_world)[BOARD_H][BOARD_W])
{
	int new_x = new_coords.first;
	int new_y = new_coords.second;
	// update old position
	true_world[old_x][old_y] = VISITED;
	// update new position
	true_world[new_x][new_y] = SUB;
}

std::pair<int, int> update_position(std::string &move, int &x, int &y)
{
	if (move == "moveRight")
	{ // moving right is col + 1
		return {x, y + 1};
	}
	else if (move == "moveLeft")
	{ // moving left is col - 1
		return {x, y - 1};
	}
	else if (move == "moveUp")
	{ // moving up is row - 1
		return {x - 1, y};
	}
	else if (move == "moveDown")
	{ // moving down is row + 1
		return {x + 1, y};
	}
	std::cerr << "update_position found invalid move: " << move << std::endl;
	return {x, y};
}

void update_directions(std::queue<std::string> &q)
{
	std::ifstream pat_output(homeDir + "/catkin_ws/src/3806ict_assignment_3/pat/output.txt");
	if (!pat_output.is_open())
	{
		std::cerr << "Failed to open pat output file!" << std::endl;
		return;
	}
	// remove all current direcitons
	while (!q.empty())
		q.pop();
	std::string line;
	std::string move;
	while (getline(pat_output, line))
	{
		if (line[0] == '<') // correct line
		{
			std::istringstream ss(line);
			ss >> move; // skip <init
			// each move will be preceded by " -> "
			while (ss >> move) // while move preceded by " -> "
			{
				// now move has current move
				ss >> move;
				q.push(move);
			}
			// removing '>' from last move
			std::string &last_move = q.back();
			last_move.pop_back();
			break;
		}
	}
	pat_output.close();
	return;
}

void generate_world(int (&world)[BOARD_H][BOARD_W], int num_survivors, int num_hostiles)
{
	// init world with EMPTY
	for (int i = 0; i < BOARD_H; ++i)
	{
		for (int j = 0; j < BOARD_W; ++j)
		{
			world[i][j] = EMPTY;
		}
	}

	// place sub in top right corner (origin)
	world[SUB_START_X][SUB_START_Y] = SUB;

	// place survivors randomly
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_int_distribution<int> rowDist(0, BOARD_H - 1);
	std::uniform_int_distribution<int> colDist(0, BOARD_W - 1);

	int placed = 0;
	while (placed < num_survivors)
	{
		int rand_row = rowDist(gen);
		int rand_col = colDist(gen);
		if (world[rand_row][rand_col] == EMPTY)
		{
			world[rand_row][rand_col] = SURVIVOR;
			placed++;
		}
	}

	// place hostiles randomly
	placed = 0;
	while (placed < num_hostiles)
	{
		int rand_row = rowDist(gen);
		int rand_col = colDist(gen);
		if (world[rand_row][rand_col] == EMPTY)
		{
			world[rand_row][rand_col] = HOSTILE;
			placed++;
		}
	}
}

void generate_known_world(int (&world)[BOARD_H][BOARD_W], int &sub_x, int &sub_y)
{

	std::ofstream file(homeDir + "/catkin_ws/src/3806ict_assignment_3/pat/world.csp");
	if (!file.is_open())
	{
		std::cerr << "Failed to save the current world to world.csp" << std::endl;
		return;
	}
	ROS_INFO("opened file at ~/catkin_ws/src/3806ict_assignment_3/pat/world.csp");
	// write defines
	file << "#define Visited -1;\n";
	file << "#define Unvisited 0;\n";
	file << "#define Sub 1;\n";
	file << "#define Bomb 2;\n";
	file << "#define Survivor 3;\n\n";
	file << "#define Rows " << BOARD_H << ";\n";
	file << "#define Cols " << BOARD_W << ";\n";
	file << "#define Fuel " << BOARD_H * BOARD_W << ";\n";

	// write new array representation of world
	file << "\nvar world[Rows][Cols]:{Visited..Survivor} = [\n";
	for (int i = 0; i < BOARD_H; i++)
	{
		for (int j = 0; j < BOARD_W; j++)
		{
			if (i == BOARD_H - 1 && j == BOARD_W - 1)
			{
				file << world[i][j];
			}
			else
			{
				file << world[i][j] << ", ";
			}
		}
		file << "\n";
	}
	file << "];\n\n";

	file << "// Position of sub\n";
	file << "var xpos:{0..Rows-1} = " << sub_x << ";\n";
	file << "var ypos:{0..Cols-1} = " << sub_y << ";\n";
	// file << "// Survivors onboard\n";
	// file << "var survivors_onboard:{0..99} = " << survivorsCollected << ";\n";
	// file << "var xpos:{0..Cols} = " << new_row << ";\n";

	file.close();
	// std::cout << "Known environment created.\n" << std::endl;
}

std::vector<int> translate_world(int (&true_world)[BOARD_H][BOARD_W])
{
	std::vector<int> vec(BOARD_W * BOARD_H, 0);
	for (int i = 0; i < BOARD_H; i++)
		for (int j = 0; j < BOARD_W; j++)
			vec[i * BOARD_W + j] = true_world[i][j];
	return vec;
}

int main(int argc, char *argv[])
{
	// init ros
	ros::init(argc, argv, "testing");
	ros::NodeHandle n;

	// create client
	ros::ServiceClient client = n.serviceClient<assignment_3::UpdateGrid>("/update_grid");
	ros::ServiceClient sensorClient = n.serviceClient<assignment_3::Sensors>("/SensorReadings");
	assignment_3::UpdateGrid srv;
	assignment_3::Sensors sensor_srv;

	std_msgs::Int32MultiArray temp_grid;

	int true_world[BOARD_H][BOARD_W];
	int current_world[BOARD_H][BOARD_W];
	// initialise all to 0
	for (int i = 0; i < BOARD_H; i++)
		for (int j = 0; j < BOARD_W; j++)
			current_world[i][j] = 0;
	current_world[SUB_START_X][SUB_START_Y] = VISITED;
	int sub_x = SUB_START_X;
	int sub_y = SUB_START_Y;
	generate_world(true_world, 3, 3);

	// int array[BOARD_H][BOARD_W] = {
	//     {1, 2, 3, 0, 0, 0},
	//     {0, 0, 0, 0, 0, 0},
	//     {0, 0, 0, 0, 2, 0},
	//     {0, 0, 0, 0, 0, 0},
	//     {0, 0, 0, 0, 0, 0},
	//     {0, 2, 0, 0, 0, 0},
	// };

	// transfer the data from int array to std_msgs::Int32MultiArray to be sent via service
	// not real sure what to comment here apart from this, Jimbo did you have more to add?
	temp_grid.layout.dim.push_back(std_msgs::MultiArrayDimension());
	temp_grid.layout.dim.push_back(std_msgs::MultiArrayDimension());
	temp_grid.layout.dim[0].label = "height";
	temp_grid.layout.dim[1].label = "width";
	temp_grid.layout.dim[0].size = BOARD_H;
	temp_grid.layout.dim[1].size = BOARD_W;
	temp_grid.layout.dim[0].stride = BOARD_H * BOARD_W;
	temp_grid.layout.dim[1].stride = BOARD_W;
	temp_grid.layout.data_offset = 0;
	std::vector<int> vec(BOARD_W * BOARD_H, 0);
	for (int i = 0; i < BOARD_H; i++)
		for (int j = 0; j < BOARD_W; j++)
			vec[i * BOARD_W + j] = true_world[i][j];
	temp_grid.data = vec;
	srv.request.grid = temp_grid;

	// std_msgs::String row;
	// row.data = "OOSOOS";

	// for(int i = 0; i < 6; i ++){
	//     srv.request.grid.push_back(row);
	// }

	if (!client.call(srv))
	{
		return EXIT_FAILURE;
	}

	std::queue<std::string> q;

	// generate world.csp
	generate_known_world(current_world, sub_x, sub_y);

	// return EXIT_SUCCESS;

	// get output from pat
	std::system(PAT_CMD.c_str());

	// extract directions from output
	update_directions(q);
	std::string next_move;
	sensor_srv.request.sensorRange = 1;
	ros::Rate rate(2);

	while (true)
	{
		ROS_INFO("-- Start of cycle --");
		// get next direction
		if (q.empty())
		{
			ROS_INFO("Successfully visited all positions within search area!\nFinal internal representation of environment:");
			for (int i = 0; i < BOARD_H; i++)
			{
				for (int j = 0; j < BOARD_W; j++)
				{
					if (current_world[i][j] != VISITED)
						std::cout << " ";
					std::cout << current_world[i][j] << " ";
				}
				std::cout << std::endl;
			}
			return EXIT_SUCCESS;
		}
		next_move = std::string(q.front());
		// remove direction from queue
		q.pop();
		ROS_INFO("Next move is: %s", next_move.c_str());

		// generate new coordinates from move
		std::pair<int, int> new_coords = update_position(next_move, sub_x, sub_y);
		int new_x = new_coords.first;
		int new_y = new_coords.second;

		// check if we can go to next position
		if (current_world[new_x][new_y] == HOSTILE)
		{
			ROS_INFO("About to move into hostile, recalculating PAT directions");
			// regenerate pat directions
			generate_known_world(current_world, sub_x, sub_y);
			std::system(PAT_CMD.c_str());
			update_directions(q);
			continue;
		}

		// new position won't be intersecting a hostile, so we can move
		sensor_srv.request.newSubXIndex = new_x;
		sensor_srv.request.newSubYIndex = new_y;

		// call the sensor client
		if (!sensorClient.call(sensor_srv))
		{
			ROS_ERROR("Failed to call service sensorReadings");
			return EXIT_FAILURE;
		}

		// update our current world with the new bot position
		// update our current world with any detected hostiles
		update_world(sensor_srv, current_world);

		// update the true world with the new bot position
		// update the previous position as 'visited'
		update_true_world(sub_x, sub_y, new_coords, true_world);

		// update bot's understanding of its position
		sub_x = new_x;
		sub_y = new_y;

		// let us know that we detected a survivor
		if (sensor_srv.response.survivorDetected)
		{
			ROS_INFO("survior detected!!");
			// return EXIT_FAILURE;
		}
		// translate world to vector for multiarray
		temp_grid.data = translate_world(true_world);
		srv.request.grid = temp_grid;
		// call update_grid service
		if (!client.call(srv))
		{
			ROS_ERROR("Failed to call service updateGrid");
		}
		ROS_INFO("-- End of cycle --\n");

		rate.sleep();
	}

	/*
	int new_col = current_col + move_col;
	int new_row = current_row + move_row;
	new_world[new_row][new_col] = VISITED;

	// need sensor data somehow
	assignment_3::sensorReadings srv;
	// supply srv.req with x and y coordinates of new sub location and sensor length

	// calll sensor service
	if (!sensorClient.call(srv))
	{
		ROS_ERROR("Failed to call service sensorReadings");
	}

	// call sensorReadings and save data
	bool bombNorth;
	bool bombSouth;
	bool bombEast;
	bool bombWest;
	bool survivorDetected;
	bool survivorsCollected;
	*/

	ros::spin();
}