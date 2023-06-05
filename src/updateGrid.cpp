#include <std_msgs/String.h>
#include <ros/ros.h>
#include <streambuf>
#include <fstream>
#include <map>
#include "geometry_msgs/Point.h"
#include "3806ict_assignment_3/UpdateGrid.h"
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/DeleteModel.h"
#include "gazebo_msgs/SetModelState.h"


ros::ServiceClient spawnClient;
ros::ServiceClient deleteClient;
ros::ServiceClient setClient;
std::vector<std_msgs::String> lastGrid;
geometry_msgs::Point coordinates[6][6];
int numSurvivors = 0;
int numObstacles = 0;
bool submarineSpawned = false;
// Dictionary with coordinates as key and survivor/obstacle as value
std::map<geometry_msgs::Point, std::string> objectPositions;
std::string homeDir = getenv("HOME");
std::string modelDir = homeDir + "/.gazebo/models/";

// function which takes a model type and returns a spawn model request
gazebo_msgs::SpawnModel createSpawnRequest(std::string modelType, geometry_msgs::Point position) {
    gazebo_msgs::SpawnModel spawn;
    if modelType == "S" {
        // Create a unique name
        spawn.request.model_name = "bowl" + numSurvivors;
        numSurvivors++;
        // Provide SDF or URDF
        std::string modelPath = modelDir + "bowl/model.sdf";
    } else if modelType == "X" {
        spawn.request.model_name = "cardboard_box" + numObstacles;
        numObstacles++;
        std::string modelPath = modelDir + "cardboard_box/model.sdf";
    } else if modelType == "_"{
        spawn.request.model_name = "submarine";
        std::string modelPath = modelDir + "submarine/model.sdf";
    } 
    std::ifstream t(modelPath);
    std::string modelXml((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());
    spawn.request.model_xml = modelXml;
    spawn.request.initial_pose.position = position;
    return spawn;
}
bool updateGrid(3806ict_assignment_3::UpdateGrid::Request& req, 3806ict_assignment_3::UpdateGrid::Response& res) {
    // Initialise new grid
    std::vector<std_msgs::String> newGrid;

    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            char lastChar = lastGrid[i].data.c_str()[j];
            char newChar = newGrid[i].data.c_str()[j];

            if (lastChar != newChar) {
                // Get the corresponding coordinates
                geometry_msgs::Point point = coordinates[i][j];
                gazebo_msgs::SpawnModel spawn;

                if (lastChar == 'O' && newChar == 'S') {
                    // Spawn a "bowl"
                    spawn = createSpawnRequest('S', point);
                    objectPositions[point] = spawn.request.model_name;
                    spawnClient.call(spawn);
                } else if (lastChar == 'S' && newChar == 'O') {
                    // Delete the "bowl"
                    gazebo_msgs::DeleteModel del;
                    deleteClient.call(objectPositions[point]);
                    objectPositions.erase(point);
                } else if (lastChar == 'O' && newChar == 'X') {
                    // Spawn a "Box"
                    spawn = createSpawnRequest('X', point);
                    objectPositions[point] = spawn.request.model_name;
                    spawnClient.call(spawn);
                } else if (lastChar == 'O' && newChar == '_') {
                    if (submarineSpawned) {
                        // Move the submarine
                        gazebo_msgs::SetModelState set;
                        set.request.model_state.model_name = "submarine";
                        set.request.model_state.pose.position = point;
                        setClient.call(set);
                    } else {
                        // Spawn the submarine
                        spawn = createSpawnRequest('_', point);
                        objectPositions[point] = spawn.request.model_name;
                        spawnClient.call(spawn);
                        submarineSpawned = true;
                    }
                }
            }
        }
    }
    // Update the lastGrid
    lastGrid = newGrid;
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "gazebo_object_manager");
    ros::NodeHandle n;
    // Initialize lastGrid to all O's
    std_msgs::String row;
    row.data = "OOOOOO";
    for (int i = 0; i < 6; ++i) {
        lastGrid.push_back(row);
    }
    // Initialise coordinates
    double spacing = 1.0;
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            coordinates[i][j].x = -3 + i * spacing;
            coordinates[i][j].y = -3 + j * spacing;
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