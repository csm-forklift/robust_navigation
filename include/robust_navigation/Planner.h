#include <ros/ros.h>
#include "nav_msgs/GetMap.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "message_filters/subscriber.h"
#include "tf/message_filter.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Point.h"
#include <vector>
#include <cstdio>
#include <iostream>
using namespace std;

class pathplan
{
	public:
		pathplan(); // constructor

		void init();

		//void testmap(float &origin_x,float &origin_y,int &width, int &height, int &resolution);

		void hueristic_cost(const nav_msgs::OccupancyGrid test_map_);

		void expand(vector<int> &step, const nav_msgs::OccupancyGrid test_map_);

		void Astar(const nav_msgs::OccupancyGrid test_map_); // start astar

		void poss_path();

		void intermediate_goal();

		float diagonalcost(int &x,int &y, int &angle, const nav_msgs::OccupancyGrid test_map_);
        
        double wrapToPi(double angle);

		bool acquire_robotpose();

	private:
		ros::NodeHandle node_;
		ros::Publisher path_cost_;
		ros::Publisher dilated_map;
        ros::Publisher movement_map_pub;

		ros::Publisher test_map; // test occupancy grid map for testing path plan algorithm

		ros::Publisher path;

		ros::Subscriber sub_;

		nav_msgs::Path *Astar_path_;

    	nav_msgs::OccupancyGrid path_cost_map_;
		//nav_msgs::OccupancyGrid *test_map_;
    	nav_msgs::OccupancyGrid test_map_dilated_;

        nav_msgs::OccupancyGrid movement_map_;
		//A* variables
        geometry_msgs::Point goal_;

		geometry_msgs::Point int_goal_;

		vector< vector<float> > hueristic_cost_;

        vector< vector<int> > *openset;

        vector< vector<int> > closedset; //also known as closed list

       	vector< vector<int> > *poss_path_;

        vector< vector<int> > temp_step;

        vector< vector<int> > optimal_path;

        vector< vector<int> > movement;
//
        vector< vector<float> > movement_cost;

        tf::TransformListener listener;

        struct Pose_{
			double x;
			double y;
			double heading;
		};

		Pose_ pose;

};
