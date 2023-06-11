#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <ros/ros.h>
#include <streambuf>
#include <fstream>
#include <map>
#include "geometry_msgs/Point.h"
#include "assignment_3/UpdateGrid.h"
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/DeleteModel.h"
#include "gazebo_msgs/SetModelState.h"
#include "assignment_3/HostileSensor.h"
#include "assignment_3/SurvivorSensor.h"
#include "communal_defines.cpp"

int currentGrid[BOARD_H][BOARD_W];
geometry_msgs::Point coordinates[BOARD_H][BOARD_W];

ros::ServiceClient spawnClient;
ros::ServiceClient deleteClient;
ros::ServiceClient setClient;
std::vector<std_msgs::String> lastGrid;

int numSurvivors = 0;
int numObstacles = 0;
bool submarineSpawned = false;
std::string homeDir = getenv("HOME");
std::string modelDir = homeDir + "/.gazebo/models/";

// comparison function for Point
struct ComparePoints
{
	bool operator()(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2) const
	{
		if (p1.x != p2.x)
			return p1.x < p2.x;
		if (p1.y != p2.y)
			return p1.y < p2.y;
		return p1.z < p2.z;
	}
};

// Dictionary with coordinates as key and survivor/obstacle as value
std::map<geometry_msgs::Point, std::string, ComparePoints> objectPositions;

// function which takes a model type and returns a spawn model request
gazebo_msgs::SpawnModel createSpawnRequest(int modelType, geometry_msgs::Point position)
{
	gazebo_msgs::SpawnModel spawn;
	std::string modelPath;
	if (modelType == SURVIVOR)
	{
		// Create a unique name
		spawn.request.model_name = "bowl" + std::to_string(numSurvivors); // bowl = survivor
		numSurvivors++;
		// Provide SDF or URDF
		modelPath = modelDir + "bowl/model.sdf";
	}
	else if (modelType == HOSTILE)
	{
		spawn.request.model_name = "cardboard_box" + std::to_string(numObstacles); // box = hostile
		numObstacles++;
		modelPath = modelDir + "cardboard_box/model.sdf";
	}
	else if (modelType == SUB)
	{
		spawn.request.model_name = "submarine"; // turtlebot = submarine
		modelPath = homeDir + "/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf";
	}
	std::ifstream t(modelPath);
	std::string modelXml((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());
	spawn.request.model_xml = modelXml;
	spawn.request.initial_pose.position = position;
	return spawn;
}

bool updateGrid(assignment_3::UpdateGrid::Request &req, assignment_3::UpdateGrid::Response &res)
{
	// Initialise new grid
	std_msgs::Int32MultiArray read_grid = req.grid;
	gazebo_msgs::SetModelState set;
	gazebo_msgs::DeleteModel del;
	gazebo_msgs::SpawnModel spawn;
	for (int i = 0; i < BOARD_H; ++i)
	{
		for (int j = 0; j < BOARD_W; ++j)
		{
			int oldIndex = currentGrid[i][j];
			int newIndex = read_grid.data[i * BOARD_W + j];
			if (oldIndex != newIndex)
			{
				// Get the corresponding coordinates
				geometry_msgs::Point point = coordinates[i][j];

				if (oldIndex == EMPTY && newIndex == SURVIVOR)
				{
					// Spawn a "Survivor"
					spawn = createSpawnRequest(SURVIVOR, point);
					objectPositions[point] = spawn.request.model_name;
					spawnClient.call(spawn);
				}
				if (oldIndex == SURVIVOR && newIndex == SUB)
				{
					// Delete the "Survivor"
					del.request.model_name = objectPositions[point];
					deleteClient.call(del);
					objectPositions.erase(point);
					// Move the "Sub"
					set.request.model_state.model_name = "submarine";
					set.request.model_state.pose.position = point;
					setClient.call(set);
				}
				if (oldIndex == EMPTY && newIndex == HOSTILE)
				{
					// Spawn a "Hostile"
					spawn = createSpawnRequest(HOSTILE, point);
					objectPositions[point] = spawn.request.model_name;
					spawnClient.call(spawn);
				}
				if ((oldIndex == EMPTY || oldIndex == VISITED) && newIndex == SUB)
				{
					if (submarineSpawned)
					{
						// Move the "Sub"
						ROS_INFO("Moving sub to pos: (%.0f, %.0f)", point.x, point.y);
						set.request.model_state.model_name = "submarine";
						set.request.model_state.pose.position = point;
						setClient.call(set);
					}
					else
					{
						// Spawn the "Sub"
						spawn = createSpawnRequest(SUB, point);
						objectPositions[point] = spawn.request.model_name;
						spawnClient.call(spawn);
						submarineSpawned = true;
					}
				}
				// Update the lastGrid
				currentGrid[i][j] = newIndex;
			}
		}
	}

	// Return the updated grid
	res.altered_grid = req.grid;
	return true;
}

bool hostileSensor(assignment_3::HostileSensor::Request &req, assignment_3::HostileSensor::Response &res)
{
	// set all to false
	res.bombNorth = false;
	res.bombSouth = false;
	res.bombWest = false;
	res.bombEast = false;
	res.survivorDetected = false;
	int x = req.newSubXIndex;
	int y = req.newSubYIndex;
	int range = req.sensorRange;

	// Initialize radar arrays
	res.northRadar = std::vector<int32_t>(range, 0);
	res.southRadar = std::vector<int32_t>(range, 0);
	res.eastRadar = std::vector<int32_t>(range, 0);
	res.westRadar = std::vector<int32_t>(range, 0);

	// Check if survivor detected
	if (currentGrid[x][y] == SURVIVOR)
	{
		res.survivorDetected = true;
	}

	// Check if there are bombs detected within the sensor range
	for (int i = 1; i <= range; ++i)
	{
		if (x - i >= 0 && currentGrid[x - i][y] == HOSTILE)
		{ // North
			res.bombNorth = true;
			res.northRadar[i - 1] = 1;
		}
		if (x + i < BOARD_H && currentGrid[x + i][y] == HOSTILE)
		{ // South
			res.bombSouth = true;
			res.southRadar[i - 1] = 1;
		}
		if (y - i >= 0 && currentGrid[x][y - i] == HOSTILE)
		{ // West
			res.bombWest = true;
			res.westRadar[i - 1] = 1;
		}
		if (y + i < BOARD_W && currentGrid[x][y + i] == HOSTILE)
		{ // East
			res.bombEast = true;
			res.eastRadar[i - 1] = 1;
		}
	}
	return true;
}


bool survivorSensor(assignment_3::SurvivorSensor::Request &req, assignment_3::SurvivorSensor::Response &res)
{
	// set all to false
	res.survivorNorth = false;
	res.survivorSouth = false;
	res.survivorWest = false;
	res.survivorEast = false;
	res.survivorDetected = false;
	int x = req.newSubXIndex;
	int y = req.newSubYIndex;
	int range = req.sensorRange;

	// Initialize radar arrays
	res.northRadar = std::vector<int32_t>(range, 0);
	res.southRadar = std::vector<int32_t>(range, 0);
	res.eastRadar = std::vector<int32_t>(range, 0);
	res.westRadar = std::vector<int32_t>(range, 0);

	// Check if survivor detected
	if (currentGrid[x][y] == SURVIVOR)
	{
		res.survivorDetected = true;
	}

	// Check if there are bombs detected within the sensor range
	for (int i = 1; i <= range; ++i)
	{
		if (x - i >= 0 && currentGrid[x - i][y] == SURVIVOR)
		{ // North
			res.survivorNorth = true;
			res.northRadar[i - 1] = 1;
		}
		if (x + i < BOARD_H && currentGrid[x + i][y] == SURVIVOR)
		{ // South
			res.survivorSouth = true;
			res.southRadar[i - 1] = 1;
		}
		if (y - i >= 0 && currentGrid[x][y - i] == SURVIVOR)
		{ // West
			res.survivorWest = true;
			res.westRadar[i - 1] = 1;
		}
		if (y + i < BOARD_W && currentGrid[x][y + i] == SURVIVOR)
		{ // East
			res.survivorEast = true;
			res.eastRadar[i - 1] = 1;
		}
	}
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "gazebo_object_manager");
	ros::NodeHandle n;
	// Initialize lastGrid to all O's
	for (int i = 0; i < BOARD_H; ++i)
	{
		for (int j = 0; j < BOARD_W; ++j)
		{
			currentGrid[i][j] = EMPTY;
		}
	}

	// Initialise coordinates
	double spacing = 1.0;
	for (int i = 0; i < BOARD_H; ++i)
	{
		for (int j = 0; j < BOARD_W; ++j)
		{
			coordinates[i][j].x = i * spacing;
			coordinates[i][j].y = j * spacing;
			coordinates[i][j].z = 0;
		}
	}
	// initialise set model state service
	setClient = n.serviceClient<gazebo_msgs::SetModelState>("gazebo/set_model_state");
	spawnClient = n.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
	deleteClient = n.serviceClient<gazebo_msgs::DeleteModel>("gazebo/delete_model");
	ros::ServiceServer HostileSenService = n.advertiseService("hostile_sensor", hostileSensor);
	ros::ServiceServer SurvivorSenService = n.advertiseService("survivor_sensor", survivorSensor);
	ros::ServiceServer updateGridService = n.advertiseService("update_grid", updateGrid);
	ros::spin();
	return 0;
}
