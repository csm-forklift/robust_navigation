#include <ros/ros.h>
#include <string>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <algorithm>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/Joy.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>

#include <dynamic_reconfigure/server.h>
#include <robust_navigation/GainsConfig.h>


using namespace std;

class VelocityController{

private:
	ros::NodeHandle nh_;
	ros::Subscriber update_path_sub; // subscribes to path from A*

    // FIXME: This subscriber runs the path controller whenever it receives a path while the one above simply updates the path. There should be a better way to do this.
    ros::Subscriber controller_path_sub;

	ros::Subscriber joystickoverideSubscriber_; // subscribes to deadman switch on joystick

	ros::Publisher cmd_vel_pub; // cmd vel to send to vehicle
	ros::Publisher local_path_pub; // publishing the current segment we are looking at
    // Publishers for Linear Velocity and Steering Angle
    ros::Publisher lin_vel_pub;
    ros::Publisher steer_angle_pub;

	ros::Duration delta_time;
	bool joystick_override, control_estop, proximity_stop; // deadman switch
	geometry_msgs::Point start_point,end_point; // local segment points
	double maximum_angular_velocity, maximum_linear_velocity;

	double angular_velocity, linear_velocity;
	double max_linear_vel, max_angular_vel; // robot constraints
	double goal_tol; // robot tolerance constraint
	double goal_heading;
	int Num_of_segments_ahead,sequence; // changes depends on path
	double steering_gain,cte_gain,error_gain,heading_gain,derivative_cte_gain,derivative_heading_gain; // gains for the controller
	double steering_angle;
	double cross_track_error_deadband; //stops path oscillation

	double cross_track_error,delta_cross_track_error,derivative_cross_track_error,heading_error,delta_heading_error,derivative_heading_error;
	double along_track_error;
	double previous_heading_error,previous_cross_track_error;
	double min_delta_time;
	tf::TransformListener listener;

	vector< vector<double> > local_path;
	int segment;
	struct Pose_{
		double x;
		double y;
		double heading;
	};
	Pose_ pose;
	dynamic_reconfigure::Server<robust_navigation::GainsConfig> server;
	dynamic_reconfigure::Server<robust_navigation::GainsConfig>::CallbackType f;

public:
	VelocityController() : nh_("~"), f(boost::bind(&VelocityController::parameter_callback, this, _1, _2)) {
		server.setCallback(f);
		get_params();
		//cmd_vel_pub = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",20); / use with actual turtlebot
        cmd_vel_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop",20); // use with gazebo turtlebot
		local_path_pub = nh_.advertise<nav_msgs::Path>("/local_path",20);
		update_path_sub = nh_.subscribe("/bspline_path/path",1, &VelocityController::update_path,this);
        controller_path_sub = nh_.subscribe("/bspline_path/path",1, &VelocityController::controller_loop,this);
        lin_vel_pub = nh_.advertise<std_msgs::Float32>("/controls/velocity_setpoint", 1);
        steer_angle_pub = nh_.advertise<std_msgs::Float32>("/controls/angle_setpoint", 1);

        // FIXME: This is a hacker way of trying to get the path_tracking_controller to run while still providing a callback to update the path that can be called simultaneously with running the controller


		joystickoverideSubscriber_ = nh_.subscribe("/joy",1, &VelocityController::joy_override,this);
		joystick_override = false;
		control_estop = false;
		proximity_stop = false;

        // cout << "*****************************************************" << endl;
        // cout << "1) Velocity controller constructor" << endl;
        // cout << "*****************************************************" << endl;
	}

    void controller_loop(nav_msgs::Path path)
    {
        if(path_tracking_controller()) {
            ROS_INFO("DONE!");
        }
    }

	bool path_tracking_controller(){

        // cout << "*****************************************************" << endl;
        // cout << "2) Running path_tracking_controller" << endl;
        // cout << "*****************************************************" << endl;

        // DEBUG:
        cout << "Begin control loop\n";

		int lookahead_segments, remaining_segments;
		ros::Time previous_time = ros::Time::now();

        // DEBUG:
        cout << "lookahead: " << lookahead_segments << "\n";
        cout << "remaining: " << remaining_segments << "\n";
        cout << "Time: " << previous_time << "\n";
        cout << "local size: " << local_path.size() << "\n";
        cout << "segment: " << segment << "\n";

		while(segment < local_path.size()-1){

			along_track_error = compute_along_track_error();
			if(along_track_error > goal_tol){
				ros::param::set("/goal_bool",false);
			}
			else{
				ros::param::set("/goal_bool",true);
			}
			while(along_track_error > goal_tol){

                // DEBUG:
                cout << "*****************************************************" << endl;
                std::cout << "3) Entered velocity command loop" << std::endl;
                cout << "*****************************************************" << endl;

				nh_.param("/control_panel_node/control_estop", control_estop,false);
				while(joystick_override || control_estop || proximity_stop){
					nh_.param("/control_panel_node/control_estop", control_estop,false);
					nh_.param("/proximity_check", proximity_stop,false);
				}

                // DEBUG:
                cout << "Obtaining robot pose\n";

				aquire_robotpose();

				start_point.x = local_path[segment][0] ;
				start_point.y = local_path[segment][1] ;
				end_point.x = local_path[segment+1][0] ;
				end_point.y = local_path[segment+1][1] ;

				remaining_segments = local_path.size() - segment;
				lookahead_segments = min(Num_of_segments_ahead, remaining_segments);
				double bendahead = path_curvature(pose.heading, lookahead_segments);
				double velocity_constraint = (1 - pow((bendahead/(3.14*0.6)),1.5));
				velocity_constraint = max(velocity_constraint,0.3);

                //==============================================================
                // Set Velocity Input
                //==============================================================
				linear_velocity = velocity_constraint*maximum_linear_velocity;

				// Slowing down if approaching last goal
				if (segment == local_path.size()-1){
					double segment_length = sqrt(pow((local_path[segment+1][0] - local_path[segment][0]),2) + pow((local_path[segment+1][1]- local_path[segment][1]),2));
					double approaching_velocity = (abs(along_track_error)/segment_length)*linear_velocity;
					linear_velocity = min(linear_velocity, approaching_velocity);
				}

				goal_heading = atan2(end_point.y - start_point.y, end_point.x - start_point.x);
				heading_error = goal_heading - pose.heading;
				// computing smallest angle
				if (heading_error > M_PI){heading_error = heading_error-(M_PI*2);}
				if (heading_error < -M_PI){heading_error=heading_error+(M_PI*2);}

				cross_track_error = compute_cross_track_error(pose.x, pose.y, start_point.x, start_point.y, end_point.x, end_point.y);
				 // This is to avoid oscillation of the robot around the path
				if(abs(cross_track_error) <= cross_track_error_deadband){cross_track_error = 0.0;}

				delta_time = ros::Time::now() - previous_time;
				if(delta_time > ros::Duration(min_delta_time)){
					previous_time = ros::Time::now();
					delta_cross_track_error = cross_track_error - previous_cross_track_error;
					derivative_cross_track_error = derivative_cte_gain * delta_cross_track_error/delta_time.toSec();
					previous_cross_track_error = cross_track_error;

					delta_heading_error = heading_error - previous_heading_error;
					derivative_heading_error = derivative_heading_gain * delta_heading_error/delta_time.toSec();
					previous_heading_error = heading_error;

				}

                //==============================================================
                // Set Steering Input
                //==============================================================
				steering_angle = heading_gain*heading_error + error_gain*atan2(cte_gain*cross_track_error,linear_velocity) + derivative_cross_track_error + derivative_heading_error;  //check this
				angular_velocity = steering_gain * steering_angle;
				if (angular_velocity > maximum_angular_velocity){angular_velocity = maximum_angular_velocity;}
				if (angular_velocity < -maximum_angular_velocity){angular_velocity = -maximum_angular_velocity;}


				// we need to send steering angle and velocity to the forklift.
				// we also need to make this controller go backwards.
				//true_vel = sqrt(pow(linear_velocity,2)+pow(angular_velocity,2));
				//steering_angle = angular_velocity/steering_gain;


				geometry_msgs::Twist velocity_command;
				velocity_command.linear.x = linear_velocity;
				velocity_command.linear.y= 0.0;
				velocity_command.linear.z= 0.0;
				velocity_command.angular.x = 0.0;
				velocity_command.angular.y = 0.0;
				velocity_command.angular.z = angular_velocity;
				cmd_vel_pub.publish(velocity_command);

                // Publish the raw Linear Velocity and Steering Angle
                // NOTE: because the forklift is going in the reverse direction,
                // the velocity and steering angle must be made negative.
                std_msgs::Float32 velocity_msg;
                std_msgs::Float32 steer_msg;
                velocity_msg.data = -linear_velocity;
                steer_msg.data = -steering_angle;
                lin_vel_pub.publish(velocity_msg);
                steer_angle_pub.publish(steer_msg);

				vector<geometry_msgs::PoseStamped> plan;
				ros::Time time_now = ros::Time::now();
		        for (int i = segment; i < segment + lookahead_segments; i++){
					geometry_msgs::PoseStamped local_plan;
			        tf::Quaternion quat = tf::createQuaternionFromYaw(0);
	 	            local_plan.header.seq = i;
		            local_plan.header.stamp = time_now;
		            local_plan.header.frame_id = "odom";
		            local_plan.pose.position.x = local_path[i][0];  // Latitude
		            local_plan.pose.position.y = local_path[i][1];  // Longitude
    		        local_plan.pose.orientation.x = quat.x();
			        local_plan.pose.orientation.y = quat.y();
			        local_plan.pose.orientation.z = quat.z();
			        local_plan.pose.orientation.w = quat.w();
			        plan.push_back(local_plan);
		        }
		        nav_msgs::Path localplan2publish;
				localplan2publish.poses = plan;
		    	localplan2publish.header.seq = sequence++;
		    	localplan2publish.header.stamp = ros::Time::now();
		    	localplan2publish.header.frame_id = "odom";
	      		local_path_pub.publish(localplan2publish);
	      		plan.clear();
				along_track_error = compute_along_track_error();
			}

		    segment++;
		}

		return true;
	}

	double path_curvature(double present_heading, int lookahead_segments){
	// compute the path curvature and add to the heading error between the robot heading and the path
		double lookahead_heading_deviation = 0.0;
		double init_angle = present_heading;
		double angle, diff_angle;
		for (int i = 1; i < lookahead_segments; i++){
			angle = atan2(local_path[segment+i][1] - local_path[segment][1], local_path[segment+i][0] - local_path[segment][0]);
			diff_angle = smallest_diff_angle(angle,  init_angle); // may require edit
			init_angle = angle;
			lookahead_heading_deviation = lookahead_heading_deviation + abs(diff_angle);
		}
		return lookahead_heading_deviation;
	}

	void update_path(nav_msgs::Path path){

        // DEBUG: check if this function is running
        cout << "*********************************************\n\n";
        cout << "*** UPDATING PATH ***\n\n";
        cout << "*********************************************" << endl;

		//pathIsAvailable = true;
		segment = 0;
		vector <double>  new_point;
		vector <vector <double> >  update_path;

        cout << "pre-'for' loop\n";
        cout << "path size: " << path.poses.size() << "\n";

		for (int i = 0; i < path.poses.size(); i++){
			new_point.push_back(path.poses[i].pose.position.x);
	        new_point.push_back(path.poses[i].pose.position.y);
	        update_path.push_back(new_point);
			new_point.clear();
		}
		local_path = update_path;
		update_path.clear();

        // DEBUG: check if local path was updated correctly
        cout << "Local path size: " << local_path.size() << "\n";

		// if(path_tracking_controller()){
		// 	ROS_INFO("DONE!");
		// }
	}
	double compute_cross_track_error(double x0, double y0, double x1, double y1, double x2, double y2){
		double Dx, Dy, numerator, denominator, xte;
		Dx = x2 - x1;
		Dy = y2 - y1;
		numerator = Dy*x0 - Dx*y0 - x1*y2 + x2*y1;
		denominator = sqrt(Dx*Dx + Dy*Dy);
		xte = numerator/denominator;
		return xte;
	}
	double compute_along_track_error(){
		double L1, L2, L3;
	 	double theta, along_track_error;
		 L1 = sqrt(pow((pose.x - local_path[segment+1][0]),2) + pow((pose.y - local_path[segment+1][1]),2));
		 L2 = sqrt(pow((local_path[segment+1][0] - local_path[segment][0]),2) + pow((local_path[segment+1][1] - local_path[segment][1]),2));
		 L3 = sqrt(pow((pose.x - local_path[segment][0]),2) + pow((pose.y - local_path[segment][1]),2));
		 theta = acos(-(L3*L3 - L1*L1 - L2*L2)/(2*L1*L2));
		 along_track_error = L1*cos(theta);
		 return along_track_error;
	}
	bool aquire_robotpose(){
		tf::StampedTransform odom_base_transform;
		tf::Matrix3x3 mat;
		double roll,pitch,yaw;
		try{
			listener.lookupTransform("odom","base_link", ros::Time(0), odom_base_transform);
		}
		catch(tf::TransformException &ex){
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
	double smallest_diff_angle(double theta2, double theta1){
		// create a quaternion for each angle
		// then multiply them with the other which is addition in quaternion
		// convert the result back to angle
		double result;
		tf::Quaternion quat2, quat1;
		quat1.setRPY(0.0, 0.0, theta1);
		quat2.setRPY(0.0, 0.0, theta2);
		return result=quat2.angleShortestPath(quat1);
	}
	void get_params(){
		//nh_.param("maximum_linear_velocity", maximum_linear_velocity, 1.0);
        nh_.param("maximum_linear_velocity", maximum_linear_velocity, 2.0);
		nh_.param("maximum_angular_velocity",maximum_angular_velocity, 1.92);
		nh_.param("Num_of_segments_ahead", Num_of_segments_ahead, 1);
		nh_.param("goal_tolerance", goal_tol, 0.3);
		nh_.param("steering_gain",steering_gain, 1.0);
	    nh_.param("error_gain",error_gain,1.0);
	    nh_.param("cte_gain", cte_gain, 1.0);
		nh_.param("heading_gain", heading_gain, 1.0);
		nh_.param("derivative_cte_gain",derivative_cte_gain, 1.0);
		nh_.param("derivative_heading_gain", derivative_heading_gain, 1.0);
		nh_.param("cross_track_error_deadband",cross_track_error_deadband, 0.3);
		nh_.param("min_delta_time",min_delta_time,0.1);
	}
	void parameter_callback(robust_navigation::GainsConfig &config, uint32_t level){
		maximum_linear_velocity =config.maximum_linear_velocity;
		maximum_angular_velocity = config.maximum_angular_velocity;
		goal_tol =config.goal_tol;
		Num_of_segments_ahead = config.num_of_segments_ahead;
		steering_gain = config.steering_gain;
	    error_gain = config.error_gain;
	    cte_gain =config.cte_gain;
		heading_gain = config.heading_gain;
		cross_track_error_deadband =  config.cross_track_error_deadband;
		derivative_cte_gain=config.derivative_cte_gain;
		derivative_heading_gain = config.derivative_heading_gain;
		min_delta_time =config.min_delta_time;
	}
	void joy_override(const sensor_msgs::Joy joy_msg){
		if(joy_msg.buttons[4] == 1){joystick_override = true;}
		else{joystick_override = false;}
	}
};

int main(int argc, char **argv){

	ROS_INFO("initializing velocity_controller_reverse");
	ros::init(argc, argv, "velocity_controller_reverse");

	VelocityController VC;

	ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
	return 0;
}