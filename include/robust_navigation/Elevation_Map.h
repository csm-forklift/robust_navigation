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

class ElevationMap
{
	public:
		ElevationMap(); // constructor

		void testmap(int &width, int &height, float &resolution);
		bool acquire_robotpose();

	private:
		ros::NodeHandle node_;
		geometry_msgs::Point goal_;
		tf::TransformListener listener;
		ros::Publisher test_map; // test occupancy grid map for testing path plan algorithm

		nav_msgs::OccupancyGrid *test_map_;

		struct Pose_{
			double x;
			double y;
			double heading;
		};
		Pose_ pose;

};

