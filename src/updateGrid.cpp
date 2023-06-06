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

#define board_size 6
#define EMPTY 0
#define SUB 1
#define SURVIVOR 2
#define HOSTILE 3

int currentGrid[board_size][board_size];
geometry_msgs::Point coordinates[board_size][board_size];

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
struct ComparePoints {
    bool operator()(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) const {
        if (p1.x != p2.x) return p1.x < p2.x;
        if (p1.y != p2.y) return p1.y < p2.y;
        return p1.z < p2.z;
    }
};

// Dictionary with coordinates as key and survivor/obstacle as value
std::map<geometry_msgs::Point, std::string, ComparePoints> objectPositions;

// function which takes a model type and returns a spawn model request
gazebo_msgs::SpawnModel createSpawnRequest(int modelType, geometry_msgs::Point position) {
    gazebo_msgs::SpawnModel spawn;
    std::string modelPath;
    if (modelType == SURVIVOR) {
        // Create a unique name
        spawn.request.model_name = "bowl" + std::to_string(numSurvivors);
        numSurvivors++;
        // Provide SDF or URDF
        modelPath = modelDir + "bowl/model.sdf";
    } else if (modelType == HOSTILE) {
        spawn.request.model_name = "cardboard_box" + std::to_string(numObstacles);
        numObstacles++;
        modelPath = modelDir + "cardboard_box/model.sdf";
    } else if (modelType == SUB) {
        spawn.request.model_name = "submarine";
        modelPath = modelDir + "submarine/model.sdf";
    } 
    std::ifstream t(modelPath);
    std::string modelXml((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());
    spawn.request.model_xml = modelXml;
    spawn.request.initial_pose.position = position;
    return spawn;
}

bool updateGrid(assignment_3::UpdateGrid::Request& req, assignment_3::UpdateGrid::Response& res) {
    // Initialise new grid
    std_msgs::Int32MultiArray read_grid = req.grid;
    for (int i = 0; i < board_size; ++i) {
        for (int j = 0; j < board_size; ++j) {
            int oldIndex = currentGrid[i][j];
            //int newIndex = req.grid[i].row[j];
            int newIndex = read_grid.data[i*board_size + j];
            if (oldIndex != newIndex) {
                // Get the corresponding coordinates
                geometry_msgs::Point point = coordinates[i][j];
                gazebo_msgs::SpawnModel spawn;

                if (oldIndex == EMPTY && newIndex == SURVIVOR) {
                    // Spawn a "Survivor"
                    spawn = createSpawnRequest(SURVIVOR, point);
                    objectPositions[point] = spawn.request.model_name;
                    spawnClient.call(spawn);
                } else if (oldIndex == SURVIVOR && newIndex == EMPTY) {
                    // Delete the "Survivor"
                    gazebo_msgs::DeleteModel del;
                    del.request.model_name = objectPositions[point];
                    deleteClient.call(del);
                    objectPositions.erase(point);
                } else if (oldIndex == EMPTY && newIndex == HOSTILE) {
                    // Spawn a "Hostile"
                    spawn = createSpawnRequest(HOSTILE, point);
                    objectPositions[point] = spawn.request.model_name;
                    spawnClient.call(spawn);
                } else if (oldIndex == EMPTY && newIndex == SUB) {
                    if (submarineSpawned) {
                        // Move the submarine
                        gazebo_msgs::SetModelState set;
                        set.request.model_state.model_name = "submarine";
                        set.request.model_state.pose.position = point;
                        setClient.call(set);
                    } else {
                        // Spawn the submarine
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

int main(int argc, char **argv) {
    ros::init(argc, argv, "gazebo_object_manager");
    ros::NodeHandle n;
    // Initialize lastGrid to all O's
    for (int i = 0; i < board_size; ++i) {
        for (int j = 0; j < board_size; ++j) {
            currentGrid[i][j] = EMPTY;
        }
    }

    // Initialise coordinates
    double spacing = 1.0;
    for (int i = 0; i < board_size; ++i) {
        for (int j = 0; j < board_size; ++j) {
            coordinates[i][j].x = i * spacing;
            coordinates[i][j].y = j * spacing;
            coordinates[i][j].z = 0;
        }
    }
    //initialise set model state service
    setClient = n.serviceClient<gazebo_msgs::SetModelState>("gazebo/set_model_state");
    spawnClient = n.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
    deleteClient = n.serviceClient<gazebo_msgs::DeleteModel>("gazebo/delete_model");
    ros::ServiceServer service = n.advertiseService("update_grid", updateGrid);
    ros::spin();
    return 0;
}
