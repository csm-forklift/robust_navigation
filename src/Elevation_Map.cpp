#include <ros/ros.h>
#include <robust_navigation/Elevation_Map.h>
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


using namespace std; // avoid using "using namespace" in future programs

ElevationMap::ElevationMap()
{
  ROS_INFO("Generating Map");
  int width = 10;
  int height = 10;
  float resolution = 1;
  
  ElevationMap::testmap(width,height,resolution); // create map (10x10) and 1m cell dimension


}


void ElevationMap::testmap(int &width, int &height, float &resolution){ // might need to put a pose origin{
   test_map = node_.advertise<nav_msgs::OccupancyGrid>("/map",1,true);
   test_map_ = new nav_msgs::OccupancyGrid;
   test_map_->header.seq=1; //this might need to be updated depending on how many time the occupancy grid is called
   test_map_->header.stamp=ros::Time::now();
   test_map_->header.frame_id="odom"; //will be use robot odometry frame

   test_map_->info.map_load_time=ros::Time::now();
   test_map_->info.width = width;
   test_map_->info.height = height;
   test_map_->info.resolution = resolution; // 1 m per cell


   ros::Rate r(0.1);  // frequency

   while(node_.ok())
   {

      if(acquire_robotpose()) // update the robot pose
      {
        test_map_->info.origin.position.x = round(pose.x)-width*resolution/2;
        test_map_->info.origin.position.y = round(pose.y)-height*resolution/2;
      }
      else
      {
        ROS_ERROR("An error as occurred acquiring robot pose");
      }



      int p[width*height];

      for (int i=0;i<width*height;i++){

      if(i==99 || i==0){
    		p[i] = 0;
    	}
    	else if (i%3==1 && i%7==2){
    		p[i] =100;
    	}
      else if (i%5==1 && i%9==2){
        p[i] =100;
      }
      else if (i%4==1 && i%8==2){
        p[i] =100;
      }
    	else{
            p[i] = 0;
     	}
      }
    
      vector<signed char> a(p,p+width*height);
      test_map_->header.seq=test_map_->header.seq + 1;
      test_map_->data = a;
      test_map.publish(*test_map_);

      ROS_INFO("Occupany map produced");
      getchar();

      //r.sleep();

    }
      

}

bool ElevationMap::acquire_robotpose()
{
  tf::StampedTransform odom_base_transform;
  tf::Matrix3x3 mat;
  double roll;
  double pitch;
  double yaw;

  
  try{
  listener.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(4.0));
  listener.lookupTransform("/odom","/base_link", ros::Time(0), odom_base_transform);  
  }
  catch(tf::TransformException &ex)
  {
    ROS_ERROR("%s",ex.what());
    return false;
  }
  

  mat.setRotation(odom_base_transform.getRotation());
  mat.getRPY(roll, pitch, yaw);

  pose.x = odom_base_transform.getOrigin().getX();
  pose.y = odom_base_transform.getOrigin().getY();
  pose.heading = yaw;

  return true;
}




int main(int argc, char ** argv)
{
  ros::init(argc, argv, "elevation_map_node");
  ElevationMap em;


  return(0);
}



