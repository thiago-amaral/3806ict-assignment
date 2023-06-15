#include <ros/ros.h>
#include "assignment_3/UpdateGrid.h"
#include "assignment_3/Sensor.h"
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

#define SubIsHome(sub_x, sub_y) (sub_x == SUB_START_X && sub_y == SUB_START_Y)
#define SURVEY_AREA 0
#define COLLECT_SURVIVORS 1
#define GO_HOME 2
std::string homeDir = getenv("HOME");
#define PAT_EXE_DIR homeDir + "/Desktop/MONO-PAT-v3.6.0/PAT3.Console.exe"
#define PAT_PATH_CSP_EXPLORE_DIR homeDir + "/catkin_ws/src/3806ict_assignment_3/pat/explore.csp"
#define PAT_PATH_CSP_HOME_DIR homeDir + "/catkin_ws/src/3806ict_assignment_3/pat/return_home.csp"
#define PAT_PATH_CSP_COLLECT_SURVIVORS_DIR homeDir + "/catkin_ws/src/3806ict_assignment_3/pat/collect_survivors.csp"
#define PAT_OUTPUT_DIR homeDir + "/catkin_ws/src/3806ict_assignment_3/pat/output.txt"
std::string PAT_CMD_EXPLORE = "mono " + PAT_EXE_DIR + " " + PAT_PATH_CSP_EXPLORE_DIR + " " + PAT_OUTPUT_DIR;
std::string PAT_CMD_GO_HOME = "mono " + PAT_EXE_DIR + " -engine 1 " + PAT_PATH_CSP_HOME_DIR + " " + PAT_OUTPUT_DIR;
std::string PAT_CMD_COLLECT_SURVIVORS = "timeout 10s mono " + PAT_EXE_DIR + " -engine 1 " + PAT_PATH_CSP_COLLECT_SURVIVORS_DIR + " " + PAT_OUTPUT_DIR;
std::string PAT_CMD_COLLECT_SURVIVORS_DFS = "timeout 10s mono " + PAT_EXE_DIR + " " + PAT_PATH_CSP_COLLECT_SURVIVORS_DIR + " " + PAT_OUTPUT_DIR;

void detect_hostiles(assignment_3::Sensor &hostile_srv, int (&curr_world)[BOARD_H][BOARD_W], int &sub_x, int &sub_y);
int detect_survivors(assignment_3::Sensor &survivor_srv, int (&curr_world)[BOARD_H][BOARD_W], int &sub_x, int &sub_y);
void update_true_world(int old_x, int old_y, std::pair<int, int> new_coords, int (&true_world)[BOARD_H][BOARD_W]);
std::pair<int, int> update_position(std::string &move, int &x, int &y);
void update_directions(std::queue<std::string> &q);
void test_generate_world(int (&world)[BOARD_H][BOARD_W]);
void generate_world(int (&world)[BOARD_H][BOARD_W], int num_survivors, int num_hostiles);
void generate_known_world(int (&world)[BOARD_H][BOARD_W], int &sub_x, int &sub_y, int &onBoard);
std::vector<int> translate_world(int (&true_world)[BOARD_H][BOARD_W]);
void regenerate_moves(int (&current_world)[BOARD_H][BOARD_W], int &sub_x, int &sub_y, int &onBoard, std::queue<std::string> &q, int &currentPath);
void execute_move(int (&current_world)[BOARD_H][BOARD_W], int (&true_world)[BOARD_H][BOARD_W], int &sub_x, int &sub_y, std::pair<int, int> &new_coords);
std_msgs::Int32MultiArray createTempGrid(int (&true_world)[BOARD_H][BOARD_W]);

int main(int argc, char *argv[])
{
	// init ros, services, and grid
	ros::init(argc, argv, "testing");
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<assignment_3::UpdateGrid>("/update_grid");
	ros::ServiceClient hostileSensorClient = n.serviceClient<assignment_3::Sensor>("/hostile_sensor");
	ros::ServiceClient survivorSensorClient = n.serviceClient<assignment_3::Sensor>("/survivor_sensor");
	assignment_3::UpdateGrid srv;
	assignment_3::Sensor hostile_srv;
	assignment_3::Sensor survivor_srv;
	std_msgs::Int32MultiArray temp_grid;

	int survivors_saved = 0;
	int survivors_seen = 0;
	int true_world[BOARD_H][BOARD_W];
	int current_world[BOARD_H][BOARD_W];
	// initialise current_world to 0
	for (int i = 0; i < BOARD_H; i++)
		for (int j = 0; j < BOARD_W; j++)
			current_world[i][j] = 0;
	current_world[SUB_START_X][SUB_START_Y] = VISITED;
	int sub_x = SUB_START_X;
	int sub_y = SUB_START_Y;
	int OnBoard = 0;
	int currentPath = SURVEY_AREA;
	generate_world(true_world, SURVIVOR_COUNT, HOSTILE_COUNT);
	// test_generate_world(true_world);

	// transfer the data from int array to std_msgs::Int32MultiArray to be sent via service
	temp_grid = createTempGrid(true_world);
	srv.request.grid = temp_grid;

	if (!client.call(srv))
	{
		return EXIT_FAILURE;
	}

	std::queue<std::string> q;
	hostile_srv.request.sensorRange = HOSTILE_DETECTION_RANGE;
	survivor_srv.request.sensorRange = SURVIVOR_DETECTION_RANGE;
	// extract information from sensors at starting position
	if (!hostileSensorClient.call(hostile_srv) || !survivorSensorClient.call(survivor_srv))
	{
		ROS_ERROR("Failed to call sensor services");
		return EXIT_FAILURE;
	}

	// update our current world with any detected hostiles or survivors
	detect_hostiles(hostile_srv, current_world, sub_x, sub_y);
	int newSurvivorsDetected = detect_survivors(survivor_srv, current_world, sub_x, sub_y);

	if (newSurvivorsDetected)
	{
		// detected a survivor, change our planning to pick them up ASAP
		ROS_INFO("New survivor detected!!");
		survivors_seen++;
		currentPath = COLLECT_SURVIVORS; // in the case we are on the way to a survivor already we generate a new survivor path.
	}

	regenerate_moves(current_world, sub_x, sub_y, OnBoard, q, currentPath);

	std::string next_move;
	ros::Rate rate(5);

	while (true)
	{
		ROS_INFO("-- Start of cycle --");

		if (SubIsHome(sub_x, sub_y) && OnBoard)
		{
			// drop off any survivors
			survivors_saved += OnBoard;
			std::cout << "Saved " << OnBoard << " survivors. Total survivors now saved: " << survivors_saved << std::endl;
			OnBoard = 0;
		}
		// get next direction
		if (q.empty())
		{
			// have we collected all survivors?
			if ((survivors_saved + OnBoard) == SURVIVOR_COUNT)
			{
				if (SubIsHome(sub_x, sub_y))
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
				else
				{ // saved all survivors, but not home yet..
					currentPath = GO_HOME;
					regenerate_moves(current_world, sub_x, sub_y, OnBoard, q, currentPath);
				}
			}
			else
			{ // still people left to be saved
				ROS_INFO("We've run out of moves, but there's still people left to be saved!");
				std::cout << "survivors seen: " << survivors_seen << " saved: " << survivors_saved << " onBoard: " << OnBoard << std::endl;
				// we know where people are
				if (survivors_seen > (survivors_saved + OnBoard))
				{
					// need a strategy to save those people
					currentPath = COLLECT_SURVIVORS;
					regenerate_moves(current_world, sub_x, sub_y, OnBoard, q, currentPath);
				}
				else // we have no idea where the survivors are, need to explore
				{
					currentPath = SURVEY_AREA;
					regenerate_moves(current_world, sub_x, sub_y, OnBoard, q, currentPath);
				}
			}
		}
		next_move = std::string(q.front());
		// remove direction from queue
		q.pop();
		ROS_INFO("Next move is: %s", next_move.c_str());

		// generate new coordinates from move
		std::pair<int, int> new_coords = update_position(next_move, sub_x, sub_y);
		int new_x = new_coords.first;
		int new_y = new_coords.second;

		// checking all conditions which would prevent us from being able to move
		// check if we're about to collide with a hostile/obstacle
		if (current_world[new_x][new_y] == HOSTILE)
		{
			ROS_INFO("About to move into hostile, recalculating PAT directions");
			// need directions which follow our current path, but avoids the hostile
			regenerate_moves(current_world, sub_x, sub_y, OnBoard, q, currentPath);
			rate.sleep();
			continue;
		}

		// if we're going to move into a survivor, it *should* have been correctly planned
		// so we are assuming that we are safe to pick them up
		if (current_world[new_x][new_y] == SURVIVOR)
		{
			ROS_INFO("About to pick up a survivor :) Hooray!");
			OnBoard++;
			std::cout << "Now have " << OnBoard << " survivors onboard" << std::endl;
		}

		// changing representations
		execute_move(current_world, true_world, sub_x, sub_y, new_coords);

		// pushing the changes to gazebo
		// translate world to vector for multiarray
		temp_grid.data = translate_world(true_world);
		srv.request.grid = temp_grid;
		// call update_grid service
		if (!client.call(srv))
			ROS_ERROR("Failed to call service updateGrid");

		// now we are at the new position in gazebo!!
		sub_x = new_x;
		sub_y = new_y;

		// call the sensors and extract information
		if (!hostileSensorClient.call(hostile_srv) || !survivorSensorClient.call(survivor_srv))
		{
			ROS_ERROR("Failed to call sensor services");
			return EXIT_FAILURE;
		}

		// observe information and make decisions
		// update our current world with detected hostiles or survivors
		detect_hostiles(hostile_srv, current_world, sub_x, sub_y);
		newSurvivorsDetected = detect_survivors(survivor_srv, current_world, sub_x, sub_y);

		if (newSurvivorsDetected)
		{
			// detected a survivor, change our planning to pick them up ASAP
			std::cout << "Info from bot: " << newSurvivorsDetected << " new survivors detected" << std::endl;
			survivors_seen += newSurvivorsDetected;
			currentPath = COLLECT_SURVIVORS;
			regenerate_moves(current_world, sub_x, sub_y, OnBoard, q, currentPath);
		}

		ROS_INFO("-- End of cycle --\n");

		rate.sleep();
	}

	ros::spin();
}

void detect_hostiles(assignment_3::Sensor &hostile_srv, int (&curr_world)[BOARD_H][BOARD_W], int &sub_x, int &sub_y)
{
	// hostile detection
	// east is col + 1
	if (hostile_srv.response.objectEast)
		for (int i = 0; i < hostile_srv.request.sensorRange; i++)
			if (hostile_srv.response.eastRadar[i])
			{
				curr_world[sub_x][sub_y + 1 + i] = HOSTILE;
				ROS_INFO("Bomb detected east!");
			}
	// west is col - 1
	if (hostile_srv.response.objectWest)
		for (int i = 0; i < hostile_srv.request.sensorRange; i++)
			if (hostile_srv.response.westRadar[i])
			{
				curr_world[sub_x][sub_y - 1 - i] = HOSTILE;
				ROS_INFO("Bomb detected west!");
			}
	// north is row - 1
	if (hostile_srv.response.objectNorth)
		for (int i = 0; i < hostile_srv.request.sensorRange; i++)
			if (hostile_srv.response.northRadar[i])
			{
				curr_world[sub_x - 1 - i][sub_y] = HOSTILE;
				ROS_INFO("Bomb detected north!");
			}
	// south is row + 1
	if (hostile_srv.response.objectSouth)
		for (int i = 0; i < hostile_srv.request.sensorRange; i++)
			if (hostile_srv.response.southRadar[i])
			{
				curr_world[sub_x + 1 + i][sub_y] = HOSTILE;
				ROS_INFO("Bomb detected south!");
			}
}

int detect_survivors(assignment_3::Sensor &survivor_srv, int (&curr_world)[BOARD_H][BOARD_W], int &sub_x, int &sub_y)
{
	// survivor detection
	int newSurvivorsDetected = 0;
	// east is col + 1
	if (survivor_srv.response.objectEast)
		for (int i = 0; i < survivor_srv.request.sensorRange; i++)
			if (survivor_srv.response.eastRadar[i])
				if (curr_world[sub_x][sub_y + 1 + i] != SURVIVOR)
				{
					newSurvivorsDetected++;
					curr_world[sub_x][sub_y + 1 + i] = SURVIVOR;
					std::cout << "New survivor detected east! x: " << sub_x << " y: " << sub_y + 1 + i << std::endl;
				}
	// west is col - 1
	if (survivor_srv.response.objectWest)
		for (int i = 0; i < survivor_srv.request.sensorRange; i++)
			if (survivor_srv.response.westRadar[i])
				if (curr_world[sub_x][sub_y - 1 - i] != SURVIVOR)
				{
					newSurvivorsDetected++;
					curr_world[sub_x][sub_y - 1 - i] = SURVIVOR;
					std::cout << "New survivor detected west! x: " << sub_x << " y: " << sub_y - 1 - i << std::endl;
				}
	// north is row - 1
	if (survivor_srv.response.objectNorth)
		for (int i = 0; i < survivor_srv.request.sensorRange; i++)
			if (survivor_srv.response.northRadar[i])
				if (curr_world[sub_x - 1 - i][sub_y] != SURVIVOR)
				{
					newSurvivorsDetected++;
					curr_world[sub_x - 1 - i][sub_y] = SURVIVOR;
					std::cout << "New survivor detected north! x: " << sub_x - 1 - i << " y: " << sub_y << std::endl;
				}
	// south is row + 1
	if (survivor_srv.response.objectSouth)
		for (int i = 0; i < survivor_srv.request.sensorRange; i++)
			if (survivor_srv.response.southRadar[i])
				if (curr_world[sub_x + 1 + i][sub_y] != SURVIVOR)
				{
					newSurvivorsDetected++;
					curr_world[sub_x + 1 + i][sub_y] = SURVIVOR;
					std::cout << "New survivor detected south! x: " << sub_x + 1 + i << " y: " << sub_y << std::endl;
				}
	return newSurvivorsDetected;
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
	// moving right is col + 1
	if (move == "moveRight")
		return {x, y + 1};
	// moving left is col - 1
	else if (move == "moveLeft")
		return {x, y - 1};
	// moving up is row - 1
	else if (move == "moveUp")
		return {x - 1, y};
	// moving down is row + 1
	else if (move == "moveDown")
		return {x + 1, y};
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

void test_generate_world(int (&world)[BOARD_H][BOARD_W])
{

	/* 4 surv, 6 host
		0  1  2  3  4  5  6  7  8  9  10 11 12 13 14 15 16 17 18 19
		----------------------------- -----------------------------
	0 | 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  1, 0, 0, 0, 0, -, 0, 0, 0, 0
	1 | 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0
	2 | 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0
	3 | 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0
	4 | 0, -, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0
	5 | 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0
	6 | 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0
	7 | 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0
	8 | 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0

	9 | 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0

	10| 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
	11| 0, 0, 0, 0, 0, 0, 0, 0, 0, -, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
	12| 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
	13| -, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
	14| 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
	15| 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -, 0, 0, 0
	16| 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
	17| 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
	18| 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
	19| 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -, 1

	*/
	// init world with EMPTY
	for (int i = 0; i < BOARD_H; ++i)
		for (int j = 0; j < BOARD_W; ++j)
			world[i][j] = EMPTY;

	// place sub in top right corner (origin)
	world[SUB_START_X][SUB_START_Y] = SUB;

	// world[0][1] = HOSTILE;
	// world[1][1] = HOSTILE;
	// world[2][1] = HOSTILE;
	// world[3][1] = HOSTILE;
	// world[4][1] = HOSTILE;
	// world[5][1] = HOSTILE;
	world[0][12] = SURVIVOR;
	world[12][12] = SURVIVOR;
	world[11][12] = SURVIVOR;
	// world[1][3] = SURVIVOR;
	// world[2][1] = SURVIVOR;
	// world[2][2] = SURVIVOR;
	// world[2][3] = SURVIVOR;
}

void generate_world(int (&world)[BOARD_H][BOARD_W], int num_survivors, int num_hostiles)
{
	// init world with EMPTY
	for (int i = 0; i < BOARD_H; ++i)
		for (int j = 0; j < BOARD_W; ++j)
			world[i][j] = EMPTY;

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

void generate_known_world(int (&world)[BOARD_H][BOARD_W], int &sub_x, int &sub_y, int &onBoard)
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
	file << "#define SUB_HOME_X " << SUB_START_X << ";\n";
	file << "#define SUB_HOME_Y " << SUB_START_Y << ";\n";
	file << "#define Rows " << BOARD_H << ";\n";
	file << "#define Cols " << BOARD_W << ";\n";
	file << "#define Fuel " << MAX_FUEL << ";\n";
	file << "#define maxCapacity " << SUB_CAP << ";\n";

	// write new array representation of world
	file << "\nvar world[Rows][Cols]:{Visited..Survivor} = [\n";
	for (int i = 0; i < BOARD_H; i++)
	{
		for (int j = 0; j < BOARD_W; j++)
		{
			if (i == BOARD_H - 1 && j == BOARD_W - 1)
				file << world[i][j];
			else
				file << world[i][j] << ", ";
		}
		file << "\n";
	}
	file << "];\n\n";

	file << "// Position of sub\n";
	file << "var xpos:{0..Rows-1} = " << sub_x << ";\n";
	file << "var ypos:{0..Cols-1} = " << sub_y << ";\n";
	file << "var onBoard:{0..maxCapacity} = " << onBoard << ";\n";
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

void regenerate_moves(int (&current_world)[BOARD_H][BOARD_W], int &sub_x, int &sub_y, int &onBoard, std::queue<std::string> &q, int &currentPath)
{
	// generate world.csp
	generate_known_world(current_world, sub_x, sub_y, onBoard);

	// get output from pat (generate output.txt)
	if (currentPath == SURVEY_AREA)
	{
		ROS_INFO("Calculating a path to survey remaining area");
		std::system(PAT_CMD_EXPLORE.c_str());
	}
	else if (currentPath == COLLECT_SURVIVORS)
	{
		ROS_INFO("Calculating a path to collect remaining survivors");
		int status = std::system(PAT_CMD_COLLECT_SURVIVORS.c_str());
		if (status < 0)
		{
			std::cout << "Error: " << strerror(errno) << '\n';
			exit(1);
		}
		else
		{
			if (WIFEXITED(status))
			{
				if (WEXITSTATUS(status) == 124)
				{
					// do alternate option
					std::system(PAT_CMD_COLLECT_SURVIVORS_DFS.c_str());
				}
				std::cout << "Program returned normally, exit code " << WEXITSTATUS(status) << '\n';
			}
			else
			{
				std::cout << "Program exited abnormaly\n";
			}
		}
	}
	else if (currentPath == GO_HOME)
	{
		ROS_INFO("Calculating a path to go home");
		std::system(PAT_CMD_GO_HOME.c_str());
	}
	else
	{
		ROS_WARN("Received unknown path command! Aborting mission");
		exit(1);
	}

	// extract directions from output.txt
	update_directions(q);
}

void execute_move(int (&current_world)[BOARD_H][BOARD_W], int (&true_world)[BOARD_H][BOARD_W], int &sub_x, int &sub_y, std::pair<int, int> &new_coords)
{
	// change internal representation (new position now visited)
	int new_x = new_coords.first;
	int new_y = new_coords.second;
	current_world[new_x][new_y] = VISITED;

	// change true representation
	true_world[sub_x][sub_y] = VISITED;
	true_world[new_x][new_y] = SUB;
}

std_msgs::Int32MultiArray createTempGrid(int (&true_world)[BOARD_H][BOARD_W])
{
	std_msgs::Int32MultiArray temp_grid;
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
	{
		for (int j = 0; j < BOARD_W; j++)
		{
			vec[i * BOARD_W + j] = true_world[i][j];
		}
	}
	temp_grid.data = vec;
	return temp_grid;
}
