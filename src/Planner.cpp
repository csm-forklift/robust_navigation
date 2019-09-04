#include <ros/ros.h>
#include <robust_navigation/Planner.h>
#include <iostream>
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "message_filters/subscriber.h"
#include <tf/transform_broadcaster.h>
#include <algorithm>
#include <vector>
#include <boost/bind.hpp>
#include <geometry_msgs/Point.h>
#include "nav_msgs/MapMetaData.h"
#include <math.h>
#include <stdlib.h>
#include <map_msgs/OccupancyGridUpdate.h>


// NEED TO FIX MAP TO ALLOW PLACING ORIGIN AT ANY LOCATION AND TO HAVE A NON-SQUARE MAP
//
// Convert from Map location to Grid location
// column = (map_x - origin_x)/resolution
// row = (map_y - origin_y)/resolution
//
// Convert from Grid location to Map location
// map_x = (column*resolution) + resolution/2 + origin_x
// map_y = (row*resolution) + resolution/2 + origin_y
//
// Need to change lines that consider bounds
//
// Lines to change: 203, 206, 212, 275, 287, 318, 319, 320, 321, 386-389, 392, 403, 421-423, 452, 454, 458, 467, 471, 493

using namespace std; // avoid using "using namespace" in future programs

pathplan::pathplan()
{
    ROS_INFO("A* Path planner starting");
    //goal_.x = 2; // get this param and store it in memory
    //goal_.y = -13;
    pathplan::init();
}


void pathplan::init(){
    sub_= node_.subscribe("/map", 1, &pathplan::Astar, this); //subscribe to occupancy grid test_map
}


void pathplan::Astar(const nav_msgs::OccupancyGrid test_map_)
{ // need to input goal state in x,y terms //need an update parameter

    node_.param("/control_panel_node/goal_x", goal_.x, 0.0);
    node_.param("/control_panel_node/goal_y", goal_.y, 0.0);
	//initialize nav_msgs::Path
    path = node_.advertise<nav_msgs::Path>("/path",1,true);

    // DEBUG: publish test maps
    /*
    path_cost_ = node_.advertise<nav_msgs::OccupancyGrid>("path_cost_map_",1,true);
    dilated_map = node_.advertise<nav_msgs::OccupancyGrid>("test_map_dilated_",10,true);
    movement_map_pub = node_.advertise<nav_msgs::OccupancyGrid>("movement_map",1,true);
    */

    //initialize Astar nav_msg path
    Astar_path_ = new nav_msgs::Path;
    //int *seq = new seq(0);
    Astar_path_->header.seq=1;
    Astar_path_->header.stamp=ros::Time::now();
    Astar_path_->header.frame_id="/odom"; // change this to baselink

    if (!acquire_robotpose()){
        ROS_INFO("NO ROBOT POSE AVAILABLE");
        return;
    }

    // ROS_INFO("Final Goal: x=%f, y=%f", goal_.x, goal_.y);

    //Map parameters
    // For a robot-centric map, the origin should be the robot's pose
    // For a global map, the origin should be odom position
    // float origin_x = pose.x;
    // float origin_y = pose.y;
    float origin_x = 0;
    float origin_y = 0;
    int width = test_map_.info.width;
    int height = test_map_.info.height;
    float resolution = test_map_.info.resolution;

    // DEBUG: test maps
    /*
    path_cost_map_.header.stamp = ros::Time::now();
    path_cost_map_.header.seq = 1;
    path_cost_map_.header.frame_id = "/odom";
    path_cost_map_.info.width = width;
    path_cost_map_.info.height = height;
    path_cost_map_.info.resolution=resolution;
    path_cost_map_.info.origin = test_map_.info.origin;

    movement_map_.header.stamp = ros::Time::now();
    movement_map_.header.seq = 1;
    movement_map_.header.frame_id = "/odom";
    movement_map_.info.width = width;
    movement_map_.info.height = height;
    movement_map_.info.resolution=resolution;
    movement_map_.info.origin = test_map_.info.origin;
    movement_map_.data = test_map_.data;
    */

    test_map_dilated_.header.stamp = ros::Time::now();
    test_map_dilated_.header.seq = 1;
    test_map_dilated_.header.frame_id = "/odom";
    test_map_dilated_.info.width = width;
    test_map_dilated_.info.height = height;
    test_map_dilated_.info.resolution=resolution;
    test_map_dilated_.info.origin = test_map_.info.origin;
    test_map_dilated_.data = test_map_.data;

    //initialize hueristic_cost, movement, movement_cost, possible_paths, openset and closet used to calculate A* Path

    poss_path_ = new vector<vector<int> >;
    openset = new vector<vector<int> >;
    movement_cost.resize(height , vector<float> (width,0));
    movement.resize(height , vector<int> (width,10));
    hueristic_cost_.resize(height , vector<float> (width,0));

    pathplan::poss_path(); //create possible movement array i.e (1,0),(0,1),(1,1),(-1,-1) etc..

    pathplan::hueristic_cost(test_map_);//euclidean distance to goal (calculates an intermediate goal)

    // Origin of the path
    vector<int> origin;
    // origin.push_back(0);
    // origin.push_back(0);
    // Since we now use a global map, we must use the robot's current position as the origin (converted into "grid cell" space)
    origin.push_back((int)((pose.x - origin_x)/resolution));
    origin.push_back((int)((pose.y - origin_y)/resolution));
    openset->push_back(origin); //add the origin

    ROS_INFO("Robot Position: x=%f, y=%f",pose.x,pose.y); // Origin corresponds to odometry position of the robot
    ROS_INFO("Robot grid position: x=%d, y=%d", origin[0], origin[1]);

    bool goal_reached=0;
    int temp_iterator_;
    float total_cost_;
    int x_1_;
    int y_1_;

    while(goal_reached==0){
        float temp_cost_=100000000;
        float Best_hueristic_cost_=100000000;
        //node_.param("/goal_bool", goal_reached, false);

        // // DEBUG: check openset size and goal bool
        // cout << "openset size: " << openset->size() << endl;
        // cout << "goal: " << goal_reached << endl;

        if (openset->size()==0 && goal_reached==0){
            ROS_INFO("NO PATH TO FINAL OR INTERMEDIATE GOAL!");
            //ros::param::set("/control_panel_node/control_estop",true);
            optimal_path.clear();
            movement_cost.clear();
            movement.clear();
            hueristic_cost_.clear();
            delete poss_path_;
            delete openset;
            delete Astar_path_;
            return;
        }

        // ROS_INFO("EXPLORING OPENSET");
        for(int i=0;i<openset->size();i++){
            x_1_=(*openset)[i][0];
            y_1_=(*openset)[i][1];
            // ROS_INFO("EXPLORING %d of %ld IN THE OPENSET AT POINT (%d, %d)", i,openset->size()-1,x_1_, y_1_);
            // ROS_INFO("Comparing against intermediate goal: (%f, %f)", int_goal_.x, int_goal_.y);

            // FIXME: need to assign goal tolerances as parameters rather than hardcoded numbers
            double x_1_world = x_1_*resolution + origin_x;
            double y_1_world = y_1_*resolution + origin_y;
            // ROS_INFO("World position (%f, %f), Goal: (%f, %f)", x_1_world, y_1_world, int_goal_.x, int_goal_.y);
            if (abs(x_1_world - int_goal_.x) <= resolution/2 && abs(y_1_world - int_goal_.y) <= resolution/2){ //checking if we have reached intermediate goal point

                ROS_INFO("(x, y) world: (%f, %f)", x_1_world, y_1_world);

                if(temp_step.size()>=1){
                    temp_step.erase(temp_step.begin()+0);
                }
                goal_reached = 1;
                vector<int> pushback;
                pushback.push_back((int_goal_.x - origin_x)/resolution);
                pushback.push_back((int_goal_.y - origin_y)/resolution);
                temp_step.push_back(pushback);
                temp_iterator_=i;

                // FIXME: need to assign goal tolerances as parameters rather than hardcoded numbers
                if (abs(int_goal_.x - goal_.x) < resolution && abs(int_goal_.y - goal_.y) < resolution){
                    // ROS_INFO("FINAL GOAL REACHED! NEED ANOTHER GOAL");
                    // ROS_INFO("PATH TO FINAL GOAL CALCULATED");
		            // node_.setParam("/goal_bool",true);
                }
                else{
                    // ROS_INFO("PATH TO INTERMEDIATE GOAL CALCULATED!");
            		// node_.setParam("/goal_bool",false);
                    // ros::param::set("/control_panel_node/control_estop",false);
                }
                break;
            }
            total_cost_=movement_cost.at(x_1_+width/2).at(y_1_+height/2)+hueristic_cost_.at(x_1_+width/2).at(y_1_+height/2);//movement+grid cost
            // ROS_INFO("Openset Size: %ld, Temp Iterator: %d, Total_cost=%f, x=%d, y=%d",openset->size(),temp_iterator_,total_cost_,x_1_,y_1_);

            if ( (total_cost_ == temp_cost_ && hueristic_cost_.at(x_1_+width/2).at(y_1_+height/2) <= Best_hueristic_cost_) || (total_cost_<temp_cost_ && goal_reached==0)) {
                if(temp_step.size()>=1){
                    temp_step.erase(temp_step.begin()+0);
                }
                temp_iterator_=i; //record the iterator to delete it from the open list later
                temp_cost_ = total_cost_;
                Best_hueristic_cost_ = hueristic_cost_.at(x_1_+width/2).at(y_1_+height/2);
                vector<int> pushback;
                pushback.push_back(x_1_);
                pushback.push_back(y_1_);
                temp_step.push_back(pushback);
                // ROS_INFO("ENTERED");
            }
        }
        //ROS_INFO("SELECTED: Total_cost=%f, x=%d, y=%d",total_cost_,temp_step[0][0],temp_step[0][1]);
        openset->erase(openset->begin()+temp_iterator_); //need to remove the step being expanded upon in later steps
        expand(temp_step[0], test_map_); // expand on all possible steps from the the least total step and and store them in the openset vector
        closedset.push_back(temp_step[0]); // add to closedset
        temp_step.clear();

        // ROS_INFO("Bottom of while loop");
        // ROS_INFO("goal: %d", goal_reached);
    }

    cout << "After While Loop\n";

    int i = (int_goal_.x - origin_x)/resolution;
    int j = (int_goal_.y - origin_y)/resolution;
    //ROS_INFO("Goal: x=%d y=%d",i,j);
    vector<int> pushback;
    pushback.push_back(i);
    pushback.push_back(j);
    optimal_path.push_back(pushback);
    pushback.clear();
    bool reached=0;

    //back track to the origin
    // ROS_INFO("Beginning backtracking search for optimal path");
    while(reached==0){

        // // FIXME: check out movement vector
        // cout << "movement (" << movement.size() << "x" << movement[0].size() << "):" << endl;
        // for (int row = movement.size()-1; row >= 0; --row) {
        //     for (int col = movement[0].size()-1; col >= 0; --col) {
        //         if (col == 0) {
        //             cout << movement[row][col] << endl;
        //         }
        //         else {
        //             cout << movement[row][col] << ", ";
        //         }
        //     }
        // }
        // cout << "i,j: (" << i << "," << j << ")" << endl;
        // cout << "coords i,j: (" << i+width/2 << "," << j+height/2 << ")" << endl;

        // Check if 'i' and 'j' produce valid indices
        // if (((i + width/2) < 0 || (i + width/2) >= movement.size()) || ((j + height/2) < 0 || (j + height/2) >= movement.at(0).size())) {
        //     // Index out of range
        //     ROS_INFO("COULD NOT BACKTRACK TO ORIGIN");
        //     optimal_path.clear();
        //     movement_cost.clear();
        //     movement.clear();
        //     hueristic_cost_.clear();
        //     delete poss_path_;
        //     delete openset;
        //     delete Astar_path_;
        //     return;
        // }
        // Check if current movement is invalid (value of 10 means no direction)
        if (movement.at(i+width/2).at(j+height/2) == 10) {
            ROS_INFO("COULD NOT BACKTRACK TO ORIGIN");
            optimal_path.clear();
            movement_cost.clear();
            movement.clear();
            hueristic_cost_.clear();
            delete poss_path_;
            delete openset;
            delete Astar_path_;
            return;
        }

        int move_1 = movement.at(i+width/2).at(j+height/2);
        // ROS_INFO("Movement: %d", move_1);
        int n = 45*move_1;
        float x = cos(n*M_PI/180);
        float y = sin(n*M_PI/180);
        if (fabs(x) == float(cos(45*M_PI/180)))
        {
            x = x / cos(45*M_PI/180);
            y = y / sin(45*M_PI/180);
        }
        int i1 = i - int(x);
        int j1 = j - int(y);
        // ROS_INFO("STEP:x=%d, y=%d, MOVE: x=%f, y=%f", i,j,x,y);
        i = i1;
        j = j1;
        if((i==origin[0]) && (j==origin[1])){
            // ROS_INFO("Path to origin found");
            // ROS_INFO("(i,j), (origin), (position): (%d,%d), (%d,%d), (%f,%f)", i, j, origin[0], origin[1], origin[0]*resolution + origin_x, origin[1]*resolution + origin_y);
            reached = 1;
        }
        pushback.push_back(i);
        pushback.push_back(j);
        optimal_path.push_back(pushback);
        pushback.clear();
    }

    Astar_path_->poses.resize(optimal_path.size());

    /*
    path_cost_map_.data.resize(test_map_.data.size());

    // DEBUG: This creates a cost map for visual debugging in rviz
    for(int i=0;i<test_map_.data.size();i++){
          float x = i%(width) - width/2;
          float y = i/(height) - height/2;
          float m_cost = movement_cost.at(x+width/2).at(y+height/2);
          float h_cost = hueristic_cost_.at(x+width/2).at(y+height/2);
          path_cost_map_.data[i]=(m_cost);
    }
    */

    /*
    // DEBUG: This creates a map representing the possible movements for generating the optimal path
    int num_columns = movement.at(0).size();
    for (int row = 0; row < movement.size(); ++row) {
        for (int col = 0; col < movement[0].size(); ++col) {
            movement_map_.data.at(row*num_columns + col) = movement.at(col).at(row);
        }
    }
    */

    for(int i=optimal_path.size()-1;i>=0;i--){
        //(*Astar_path_).poses[optimal_path.size()-1-i].header.seq=i;
        (*Astar_path_).poses[optimal_path.size()-1-i].header.stamp=ros::Time::now();
        (*Astar_path_).poses[optimal_path.size()-1-i].header.frame_id="/odom";
        (*Astar_path_).poses[optimal_path.size()-1-i].pose.position.x=optimal_path[i][0]*resolution+origin_x;
        (*Astar_path_).poses[optimal_path.size()-1-i].pose.position.y=optimal_path[i][1]*resolution+origin_y;
        //optimal_path[i].clear();
        // ROS_INFO("OPTIMAL PATH: x=%f, y=%f", optimal_path[i][0]*resolution+origin_x, optimal_path[i][1]*resolution+origin_y);
    }

    //----- Post-Processing -----//
    // Take the robots current position and replace the first position in the path with this value. Then take the exact goal position and replace the final path position with this value.
    Astar_path_->poses[0].pose.position.x = pose.x;
    Astar_path_->poses[0].pose.position.y = pose.y;
    Astar_path_->poses[Astar_path_->poses.size() - 1].pose.position.x = goal_.x;
    Astar_path_->poses[Astar_path_->poses.size() - 1].pose.position.y = goal_.y;
    //----- Post-Processing -----//

    path.publish(*Astar_path_);
    //path_cost_.publish(path_cost_map_);
    //dilated_map.publish(test_map_dilated_);
    //movement_map_pub.publish(movement_map_);

    optimal_path.clear();
    movement_cost.clear();
    movement.clear();
    hueristic_cost_.clear();
    delete poss_path_;
    delete openset;
    delete Astar_path_;
    return;
}

void pathplan::hueristic_cost(const nav_msgs::OccupancyGrid test_map_)
{ //calulates intermediate goal (Assumes resolution is 1 except to calulate the hueristic cost)
    // For a robot-centric map, the origin should be the robot's pose
    // For a global map, the origin should be odom position
    // float origin_x = pose.x;
    // float origin_y = pose.y;
    float origin_x = 0;
    float origin_y = 0;

    int width = test_map_.info.width;
    int height = test_map_.info.height;
    //float map_origin_x = test_map_.info.origin.position.x;
    //float map_origin_y = test_map_.info.origin.position.y;
    float resolution = test_map_.info.resolution;

    float best_hueristic_cost_ = 100000000;//sqrt(pow(map_origin_x-goal_.x,2) + pow(map_origin_y-goal_.y,2));

    // if ( abs(goal_.x-origin_x)  > width/2 || abs(goal_.y-origin_y) > height/2){ // if the goal point is outside the map then we need to find intermediate points
    for (int i=0;i<(width)*(height);i++) // calculate euclidean distance // 100 = map_width * map_height
    {
        int x = i%(width) - width/2;
        int y = i/(height) - height/2;
        // int y = i/width - height/2;
        float cost =  sqrt(pow((x*resolution+origin_x)-goal_.x,2) + pow((y*resolution+origin_y)-goal_.y,2));

        // hueristic_cost_.at(i%width).at(int(i/width)) = cost;
        hueristic_cost_.at(i%height).at(int(i/width)) = cost; // euclidean + occupied cost
        // ROS_INFO("x=%d,y=%d,cost=%f",x,y,cost);


        
        float radius_cost = (2/resolution); //dilate cost around the obstacle
        float radius_obstacle = (0.2/resolution); // dilate obstacle
        if (test_map_.data[i]==100){ //dilate cost
            for(int j=0;j<int(radius_cost);j++){
                for (int k=0;k<poss_path_->size();k++){

                int x_1 = x + j*(*poss_path_)[k][0];
                int y_1 = y + j*(*poss_path_)[k][1];
                    if (x_1 > -width/2 && x_1<width/2 && y_1 > -height/2 && y_1<height/2){
                        // // This section adds an additional dilation to the map which may be unnecessary
                        // if(j<radius_obstacle){
                        //     test_map_dilated_.data[(x_1 + width/2) + width*(y_1+height/2)]=100;
                        // }
                        
                        // THIS SECTION WAS CHANGED ON JANUARY 14th for the demo to work we commented out the following line and instead set it to 0
                        movement_cost.at(x_1+width/2).at(y_1+height/2)=5*(radius_cost-j); //diminishing cost
                        // movement_cost.at(x_1+width/2).at(y_1+height/2)=0;
                    }
                }
            }
        }

        if (cost < best_hueristic_cost_ && test_map_dilated_.data[i] != 100  &&  x > -width/2 && x<width/2 && y > -height/2 && y<height/2){
            int_goal_.x = x*resolution + origin_x;
            int_goal_.y = y*resolution + origin_y;
            best_hueristic_cost_= cost;
        }
    }
    // ROS_INFO("Intermediate Goal: x=%f, y=%f", int_goal_.x, int_goal_.y);
}


void pathplan::expand(vector<int> &step, const nav_msgs::OccupancyGrid test_map_)
{
    // For a robot-centric map, the origin should be the robot's pose
    // For a global map, the origin should be odom position
    // float origin_x = pose.x;
    // float origin_y = pose.y;
    float origin_x = 0;
    float origin_y = 0;
    int width = test_map_.info.width;
    int height = test_map_.info.height;
    float resolution = test_map_.info.resolution;
    vector<int> b;
    // ROS_INFO("possible paths: %d", (int)poss_path_->size());
    for (int i=0;i<poss_path_->size();i++){
        int x = step[0] + (*poss_path_)[i][0];
        int y = step[1] + (*poss_path_)[i][1];
        b.push_back(x);
        b.push_back(y);
        if (x > -width/2 && x<width/2 && y > -height/2 && y<height/2){ //Only expand on nodes inside the domain and unoccupied
            if (test_map_dilated_.data[(x + width/2) + width*(y+height/2)]!=100 && movement.at(x+width/2).at(y+height/2)==10){ //Only expand on nodes that are not explored
                openset->push_back(b); // add the expanded node to the openset
                movement.at(x+width/2).at(y+height/2)=i; //capture movement for backtracking for optimal path later
                int n = 45*i;
                float pose_change_error = abs(n*M_PI/180-pose.heading);
		            float unexplored_cost=0;
		            if (test_map_.data[(x + width/2) + width*(y+height/2)] == -1){
			      		   unexplored_cost = 1.0;
		       	    }
			          else{
				           unexplored_cost = 0.0;
		    	      }
                	// add cost of the expanded cell to the movement_cost 2zD array
                    if(i%2 == 1){
                        float diagonal_cost_=pathplan::diagonalcost(x,y,i, test_map_);
                        movement_cost.at(x+width/2).at(y+height/2)=  unexplored_cost + movement_cost.at(x+width/2).at(y+height/2)+movement_cost.at(step[0]+width/2).at(step[1]+height/2) +  1.4 + diagonal_cost_  ; //make these costs variable
                        //ROS_INFO("Expanded: x=%d, y=%d, cost=%f",x,y,movement_cost.at(x+width/2).at(y+height/2));
                    }
                    else{
                        movement_cost.at(x+width/2).at(y+height/2)=  unexplored_cost + movement_cost.at(x+width/2).at(y+height/2)+movement_cost.at(step[0]+width/2).at(step[1]+height/2) +  1 ; //make these costs variable
                        //ROS_INFO("Expanded: x=%d, y=%d, cost=%f",x,y,movement_cost.at(x+width/2).at(y+height/2));
                    }
            }
        }
        b.clear();
    }
}


float pathplan::diagonalcost(int &x,int &y, int &angle, const nav_msgs::OccupancyGrid test_map_)
{
    float n=90*(angle-1);
    float x_1= 0.0;
    float y_1 = 0.0;
    float diagonal_cost=0.0;

    int width = test_map_.info.width;
    int height = test_map_.info.height;
    for(int i=0;i<2;i++){
        x_1 = cos(n*M_PI/180);
        y_1 = sin(n*M_PI/180);
        if(int(x_1+x)> -width/2 && int(x_1+x)< width/2 && int(y_1+y)> -height/2 && int(y_1+y)< height/2){
            x_1 = x_1 + x;
            y_1 = y_1 + y;
            if(test_map_dilated_.data[(x_1 + width/2) + width*(y_1+height/2)]==100.0){
                diagonal_cost = diagonal_cost + 10;
            }
        }
        n = n + 90.0;
    }
    return diagonal_cost;
}

void pathplan::poss_path()
{
    vector<int> pushback;
    int n;
    float x,y;
    for (int i=0;i<=7;i++){
        n = 45*i;
        x = cos(n*M_PI/180);
        y = sin(n*M_PI/180);
        // This section keeps the magnitude of x and y as 1 so that the points representing the 8 possible directions are the middle of the edges and corners of a square.
        if (fabs(x) == float(cos(45*M_PI/180)))
        {
            x = x / cos(45*M_PI/180);
            y = y / sin(45*M_PI/180);
        }
        pushback.push_back(x);
        pushback.push_back(y);
        poss_path_->push_back(pushback);
        pushback.clear();
    }
}

double pathplan::wrapToPi(double angle)
{
    angle = fmod(angle + M_PI, 2*M_PI);
    if (angle < 0) {
        angle += 2*M_PI;
    }
    return (angle - M_PI);
}

bool pathplan::acquire_robotpose()
{
    tf::StampedTransform odom_base_transform;
    tf::Matrix3x3 mat;
    double roll;
    double pitch;
    double yaw;

    try {
        // NOTE: currently set to 'sensor_link' so that the path follows the rear of the forklift as it drives backwards. this helps to keep from having steering troubles with using the front as base_link and then steering from the back.
        listener.waitForTransform("/odom", "/sensor_link", ros::Time(0), ros::Duration(0.5));
        listener.lookupTransform("/odom", "/sensor_link", ros::Time(0), odom_base_transform);
    }
    catch(tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        return false;
    }

    mat.setRotation(odom_base_transform.getRotation());
    mat.getRPY(roll, pitch, yaw);

    pose.x = odom_base_transform.getOrigin().getX();
    pose.y = odom_base_transform.getOrigin().getY();
    // Flip the yaw angle by 180 degrees because we are driving backwards
    //pose.heading = wrapToPi(yaw + M_PI);

    pose.heading = yaw;

    return true;
}



int main(int argc, char ** argv)
{
  ros::init(argc, argv, "planner_node");
  pathplan pp;

  ros::spin();

  return(0);
}
