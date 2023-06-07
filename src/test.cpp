#include <ros/ros.h>
#include "assignment_3/UpdateGrid.h"
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <vector>

#define H 6
#define W 6

int main(int argc, char *argv[]){
    ros::init(argc, argv, "testing");
    ros::NodeHandle n;

    ros::ServiceClient client= n.serviceClient<assignment_3::UpdateGrid>("/update_grid");
    assignment_3::UpdateGrid srv;

    std_msgs::Int32MultiArray temp_grid;
    int array[H][W] = {
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 2, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 2, 0, 0, 0, 0},
    };

    temp_grid.layout.dim.push_back(std_msgs::MultiArrayDimension());
    temp_grid.layout.dim.push_back(std_msgs::MultiArrayDimension());
    temp_grid.layout.dim[0].label = "height";
    temp_grid.layout.dim[1].label = "width";
    temp_grid.layout.dim[0].size = H;
    temp_grid.layout.dim[1].size = W;
    temp_grid.layout.dim[0].stride = H*W;
    temp_grid.layout.dim[1].stride = W;
    temp_grid.layout.data_offset = 0;
    std::vector<int> vec(W*H, 0);
    for(int i=0; i<H; i++)
        for(int j=0; j<W; j++)
            vec[i*W + j] = array[i][j];
    temp_grid.data = vec;    
    srv.request.grid = temp_grid;

    // std_msgs::String row;
    // row.data = "OOSOOS";

    // for(int i = 0; i < 6; i ++){
    //     srv.request.grid.push_back(row);
    // }

    if (!client.call(srv)) {
        return EXIT_FAILURE;
    }

    ros::spin();
}