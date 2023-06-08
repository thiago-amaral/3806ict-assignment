#include <ros/ros.h>
#include "assignment_3/UpdateGrid.h"
#include "assignment_3/sensorReadings.h"
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <vector>
#include <random>

#define H 6
#define W 6
#define EMPTY 0
#define SUB 1
#define SURVIVOR 2
#define HOSTILE 3

ros::ServiceClient sensorClient;

void generate_world(int (&world)[H][W], int num_survivors, int num_hostiles)
{
    // init world with EMPTY
    for (int i = 0; i < H; ++i)
    {
        for (int j = 0; j < W; ++j)
        {
            world[i][j] = EMPTY;
        }
    }

    // place sub in top right corner (origin)
    world[0][0] = SUB;

    // place survivors randomly
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> rowDist(0, H - 1);
    std::uniform_int_distribution<int> colDist(0, W - 1);

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

void generate_known_world(std::string filename, int current_row, int current_col, int move_row, int move_col,
                          std::vector<std::vector<int>> &old_world)
{
    std::vector<std::vector<int>> new_world = old_world;
    // update current position of sub
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

    // update array with new data eg:
    // new_world[row][col] = BOMB

    //
    std::ofstream file(filename);
    if (!file)
    {
        std::cerr << "Failed to open the file: " << filename << std::endl;
        return;
    }
    // write defines
    file << "#define Visited -1;\n";
    file << "#define Unvisited 0;\n";
    file << "#define Sub 1;\n";
    file << "#define Bomb 2;\n";
    file << "#define Survivor 3;\n\n";
    file << "#define Rows " << H << ";\n";
    file << "#define Cols " << W << ";\n";
    file << "#define Fuel " << H * W << ";\n";

    // write new array representation of world
    file << "\nvar world[Rows][Cols]:{Visited..Survivor} = [\n";
    for (int i = 0; i < H; i++)
    {
        for (int j = 0; j < W; j++)
        {
            if (i == H - 1 && j == W - 1)
            {
                file << new_world[i][j];
            }
            else
            {
                file << new_world[i][j] << ", ";
            }
        }
        file << "\n";
    }
    file << "];\n\n";

    file << "// Position of sub\n";
    file << "var xpos:{0..Rows} = " << new_col << ";\n";
    file << "var xpos:{0..Cols} = " << new_row << ";\n";
    file << "// Survivors onboard\n";
    file << "var survivors_onboard:{0..99} = " << survivorsCollected << ";\n";
    file << "var xpos:{0..Cols} = " << new_row << ";\n";

    file.close();
    // std::cout << "Known environment created.\n" << std::endl;
}

int main(int argc, char *argv[])
{
    // init ros
    ros::init(argc, argv, "testing");
    ros::NodeHandle n;

    // create client
    ros::ServiceClient client = n.serviceClient<assignment_3::UpdateGrid>("/update_grid");
    assignment_3::UpdateGrid srv;

    std_msgs::Int32MultiArray temp_grid;

    int array[H][W];
    generate_world(array, 3, 3);

    // int array[H][W] = {
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
    temp_grid.layout.dim[0].size = H;
    temp_grid.layout.dim[1].size = W;
    temp_grid.layout.dim[0].stride = H * W;
    temp_grid.layout.dim[1].stride = W;
    temp_grid.layout.data_offset = 0;
    std::vector<int> vec(W * H, 0);
    for (int i = 0; i < H; i++)
        for (int j = 0; j < W; j++)
            vec[i * W + j] = array[i][j];
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

    ros::spin();
}