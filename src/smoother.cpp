#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Path.h"
#include "math.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "iostream"
#include "vector"

ros::Publisher pub;
using namespace std;

int factorial (int n)
{
	if (n>1)
	return n*factorial(n-1);
	else
	return 1;
}

void callback(const nav_msgs::Path::ConstPtr& msg)
{
nav_msgs::Path smoothedpath;

// // FIXME: print header
// cout << "**************************************************" << endl;
// cout << "Frame ID of path: " << msg->header << endl;
// cout << "**************************************************" << endl;

smoothedpath.header = msg->header;

vector<geometry_msgs::Point> Waypoints;
vector<double> Slope;

Waypoints.push_back(msg->poses[0].pose.position);
Waypoints.push_back(msg->poses[1].pose.position);
Slope.push_back(((msg->poses[1].pose.position.y)-(msg->poses[0].pose.position.y))/((msg->poses[1].pose.position.x)-(msg->poses[0].pose.position.x)));
int segments = 10;

	for (int i =2; i<=msg->poses.size()-1; i++){
    	geometry_msgs::PoseStamped smoothedpose;
      	//ROS_INFO("I heard: [%f, %f]", msg->poses[i].pose.position.x, msg->poses[i].pose.position.y);
      	Slope.push_back(((msg->poses[i].pose.position.y)-(msg->poses[i-1].pose.position.y))/((msg->poses[i].pose.position.x)-(msg->poses[i-1].pose.position.x)));
      	if (Slope[i-1] == Slope[i-2]){
      		smoothedpose.pose.position = Waypoints[0];
      		smoothedpath.poses.push_back(smoothedpose);

      		segments = Waypoints.size();
      		float t = 1.0/(segments);
      		for (int j = 1; j<= segments-1; j++){
      			float tempx = 0;
        		float tempy = 0;
      			for (int k = 0; k<Waypoints.size(); k++){
      			float coeff =  (factorial(Waypoints.size()-1)/(factorial(k)*factorial((Waypoints.size()-1)-k)));
        		float minus1 = pow((1 - j*t),(Waypoints.size()-1)-k);
        		float term_t = pow(j*t,k);
        		tempx += coeff*minus1*term_t*(Waypoints[k].x);
        		tempy += coeff*minus1*term_t*(Waypoints[k].y);
      			}
      			smoothedpose.pose.position.x =tempx;
        		smoothedpose.pose.position.y =tempy;
      		    smoothedpath.poses.push_back(smoothedpose);
      		}
      		Waypoints.clear();
      		Waypoints.push_back(msg->poses[i-1].pose.position);
      		Waypoints.push_back(msg->poses[i].pose.position);
      	}
      	else if (i == msg->poses.size()-1){
      		smoothedpose.pose.position = Waypoints[0];
      		smoothedpath.poses.push_back(smoothedpose);
      		Waypoints.push_back(msg->poses[i].pose.position);

      		segments = Waypoints.size();
      		float t = 1.0/(segments);
      		for (int j = 1; j<= segments-1; j++){
      			float tempx = 0;
        		float tempy = 0;
      			for (int k = 0; k<Waypoints.size(); k++){
      			float coeff =  (factorial(Waypoints.size()-1)/(factorial(k)*factorial((Waypoints.size()-1)-k)));
        		float minus1 = pow((1 - j*t),(Waypoints.size()-1)-k);
        		float term_t = pow(j*t,k);
        		tempx += coeff*minus1*term_t*(Waypoints[k].x);
        		tempy += coeff*minus1*term_t*(Waypoints[k].y);
      			}
      			smoothedpose.pose.position.x =tempx;
        		smoothedpose.pose.position.y =tempy;
      		    smoothedpath.poses.push_back(smoothedpose);
      		}
      	}
      	else {
      		Waypoints.push_back(msg->poses[i].pose.position);
      	}



/*double t = 1.0/(msg->poses.size());
            smoothedpose.pose.position.x = pow((1-t), 3)*msg->poses[i-3].pose.position.x + 3*pow((1-t), 2)*t*msg->poses[i-2].pose.position.x + 3*(1-t)*pow(t, 2)*msg->poses[i-1].pose.position.x+pow(t,3)*msg->poses[i].pose.position.x;
	        smoothedpose.pose.position.y = pow((1-t), 3)*msg->poses[i-3].pose.position.y + 3*pow((1-t), 2)*t*msg->poses[i-2].pose.position.y + 3*(1-t)*pow(t, 2)*msg->poses[i-1].pose.position.y+pow(t,3)*msg->poses[i].pose.position.y;

	        smoothedpath.poses.push_back(smoothedpose);
	/*for(int j=msg->poses.size()-1;j>=0;j--){
          smoothedpath.poses[msg->poses.size()-1-j].header.seq=j;
	}*/

	}
        smoothedpath.poses.push_back(msg->poses[msg->poses.size()-1]);

    pub.publish(smoothedpath);
}



int main(int argc, char **argv)
{

  ros::init(argc, argv, "smoother_node");


  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("path", 1, callback);
  pub = n.advertise<nav_msgs::Path>("pathsmoothed", 5);

    ros::spin();

   return 0;
}
