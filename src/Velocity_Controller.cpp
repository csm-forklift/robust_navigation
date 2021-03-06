#include <ros/ros.h>
#include <string>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <algorithm>
#include <cmath> // for M_PI, M_PI_2
#include <sys/time.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/Joy.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>

#include <dynamic_reconfigure/server.h>
#include <robust_navigation/GainsConfig.h>


using namespace std;

class VelocityController{

private:
    // ROS Objects
	ros::NodeHandle nh_;
	ros::Subscriber update_path_sub; // subscribes to path from A*
    // FIXME: This subscriber runs the path controller whenever it receives a path while the one above simply updates the path. There should be a better way to do this.
    ros::Subscriber controller_path_sub;
	ros::Subscriber joystickoverideSubscriber_; // subscribes to deadman switch on joystick
    ros::Subscriber gear_sub; // reads in the desired gear: forward, neutral, reverse
    ros::Subscriber current_angle_sub; // reads the current steering angle, used to compared against desired setpoint
    ros::Subscriber control_mode_sub; // reads the current control mode set by the master controller
	ros::Publisher cmd_vel_pub; // cmd vel to send to vehicle
	ros::Publisher local_path_pub; // publishing the current segment we are looking at
    // Publishers for Linear Velocity and Steering Angle
    ros::Publisher lin_vel_pub;
    ros::Publisher steer_angle_pub;
	ros::Duration delta_time;
    tf::TransformListener listener;
    ros::Rate rate; // used for the inner while loop to slow it down when the current operating mode is not set to run this loop

    // Bounds
    geometry_msgs::Point start_point,end_point; // local segment points
    double maximum_angular_velocity, maximum_linear_velocity;
    double max_linear_vel, max_angular_vel; // robot constraints

    // State variables
    double steering_angle, steering_angle_min, steering_angle_max;
    double current_angle, current_angle_error_max;
	double angular_velocity, linear_velocity;
    int autonomous_deadman_button, manual_deadman_button;
	bool autonomous_deadman_on, manual_deadman_on, control_estop, proximity_stop; // deadman switch
    double timeout, timeout_start;
	double goal_heading;
    bool path_has_been_updated;
    int gear; // indicates the forklift's gear: 1 = forward, 0 = neutral, -1 = reverse

    // Controller Parameters
    int control_mode; // this node publishes only when this variable is a specific value
    vector<int> available_control_modes; // vector of possible numbers the 'control_mode' can be to turn this node on
    double steering_gain,cte_gain,error_gain,heading_gain,derivative_cte_gain,derivative_heading_gain; // gains for the controller
    double cross_track_error,delta_cross_track_error,derivative_cross_track_error,heading_error,delta_heading_error,derivative_heading_error;
	double along_track_error;
	double previous_heading_error,previous_cross_track_error;
    double cross_track_error_deadband; //stops path oscillation
    double goal_tol; // robot tolerance constraint
    double min_delta_time;
    int num_of_segments_ahead,sequence; // changes depends on path

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
	VelocityController() : nh_("~"), f(boost::bind(&VelocityController::parameter_callback, this, _1, _2)), rate(30)
    {
		server.setCallback(f);
		get_params();
		//cmd_vel_pub = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",20); / use with actual turtlebot
        cmd_vel_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop",20); // use with gazebo turtlebot
		local_path_pub = nh_.advertise<nav_msgs::Path>("/local_path",20);

        lin_vel_pub = nh_.advertise<std_msgs::Float64>("/velocity_node/velocity_setpoint", 1);
        steer_angle_pub = nh_.advertise<std_msgs::Float64>("/steering_node/angle_setpoint", 1);

        // FIXME: This is a hacker way of trying to get the path_tracking_controller to run while still providing a callback to update the path that can be called simultaneously with running the controller
		update_path_sub = nh_.subscribe("/path",1, &VelocityController::update_path,this);
        controller_path_sub = nh_.subscribe("/path",1, &VelocityController::controller_loop,this);
        gear_sub = nh_.subscribe("/velocity_node/gear", 1, &VelocityController::gearCallback, this);
        current_angle_sub = nh_.subscribe("/steering_node/filtered_angle", 1, &VelocityController::angleCallback, this);
        control_mode_sub = nh_.subscribe("/control_mode", 1, &VelocityController::controlModeCallback, this);
		joystickoverideSubscriber_ = nh_.subscribe("/joy",1, &VelocityController::joy_override,this);
		autonomous_deadman_on = false;
        manual_deadman_on = false;
        timeout_start = getWallTime();
		control_estop = false;
		proximity_stop = false;
        path_has_been_updated = false;

        //===== Print out possible values for control mode =====//
        // Pushback more numbers to allow this controller to operate in more
        // modes
        control_mode = 0; // start off with no controller operating
        available_control_modes.push_back(1); // for forward operation
        available_control_modes.push_back(2); // for reverse operation
        string message = "Available control_modes for [" + ros::this_node::getName() + "]: ";
        for (int i = 0; i < available_control_modes.size(); ++i) {
            char msg_buffer[10]; // increase size if more digits are needed
            sprintf(msg_buffer, "%d", available_control_modes.at(i));
            message += msg_buffer;
            if (i != available_control_modes.size() - 1) {
                message += ", ";
            }
            else {
                message += '\n';
            }
        }
        ROS_INFO("%s", message.c_str());

        // cout << "*****************************************************" << endl;
        // cout << "1) Velocity controller constructor" << endl;
        // cout << "*****************************************************" << endl;
	}

    void controller_loop(nav_msgs::Path path)
    {
        if (path_has_been_updated) {
            if (checkControlMode(control_mode, available_control_modes)) {
                if(path_tracking_controller()) {
                    // Stop linear velocity
                    std_msgs::Float64 velocity_msg;
                    velocity_msg.data = 0.0;
                    lin_vel_pub.publish(velocity_msg);
                    ros::param::set("/goal_bool", true);
                    ROS_INFO("DONE!");
                }
            }
        }
    }

	bool path_tracking_controller()
    {
        // // DEBUG: check we are in here
        // cout << "*****************************************************\n";
        // cout << "2) Running path_tracking_controller\n";
        // cout << "*****************************************************\n";

		int lookahead_segments, remaining_segments;
		ros::Time previous_time = ros::Time::now();

        // // DEBUG:
        // cout << "lookahead: " << lookahead_segments << "\n";
        // cout << "remaining: " << remaining_segments << "\n";
        // cout << "Time: " << previous_time << "\n";
        // cout << "local size: " << local_path.size() << "\n";
        // cout << "segment: " << segment << "\n";

		while(segment < local_path.size()-1) {

            // // DEBUG: show progress
            // cout << "Segment: " << segment << " of " << local_path.size()-1 << "\n";

			along_track_error = compute_along_track_error();
			if(along_track_error > goal_tol) {
				ros::param::set("/goal_bool",false);
			}
            /*
			else {
				ros::param::set("/goal_bool",true);
			}
            */

            // // DEBUG: check this loop
            // printf("along_track: %0.04f, goal_tol: %0.03f\n", along_track_error, goal_tol);

            // Main control loop
			while((along_track_error > goal_tol) && ros::ok()) {

                if (checkControlMode(control_mode, available_control_modes)) {
                    // // DEBUG: print along track error
                    // printf("Reverse\n");
                    // int num_segments = local_path.size()-1;
                    // printf("Segment %d of %d, error: %0.4g, tol: %0.4g\n", segment, num_segments, along_track_error, goal_tol);

                    // // DEBUG:
                    // cout << "**************************************" << endl;
                    // std::cout << "3) Entered velocity command loop" << std::endl;
                    // cout << "**************************************" << endl;

    				nh_.param("/control_panel_node/control_estop", control_estop,false);
    				while(manual_deadman_on || control_estop || proximity_stop) {
    					nh_.param("/control_panel_node/control_estop", control_estop,false);
    					nh_.param("/proximity_check", proximity_stop,false);
    				}

    				aquire_robotpose();

                    // If driving in reverse the heading must be flipped by 180 deg, if driving forward heading is the same
                    double forklift_heading;
                    if (control_mode == 1) {
                        forklift_heading = pose.heading;
                    }
                    else if (control_mode == 2) {
                        forklift_heading = wrapToPi(pose.heading + M_PI);
                    }
                    else {
                        ROS_INFO("[%s]: control mode does not match.", ros::this_node::getName().c_str());
                    }

    				start_point.x = local_path[segment][0] ;
    				start_point.y = local_path[segment][1] ;
    				end_point.x = local_path[segment+1][0] ;
    				end_point.y = local_path[segment+1][1] ;

    				remaining_segments = local_path.size() - segment;
    				lookahead_segments = min(num_of_segments_ahead, remaining_segments);

                    //==========================================================
                    // Get Steering Control Law Parameters
                    //==========================================================
    				goal_heading = atan2(end_point.y - start_point.y, end_point.x - start_point.x);
					//heading_error = goal_heading - forklift_heading; // because the forklift is driving in reverse, the heading must be flipped 180 degrees

					heading_error = lookAheadError(forklift_heading, lookahead_segments);

    				// computing smallest angle
    				if (heading_error > M_PI) {heading_error = heading_error-(M_PI*2);}
    				if (heading_error < -M_PI) {heading_error=heading_error+(M_PI*2);}

    				cross_track_error = compute_cross_track_error(pose.x, pose.y, start_point.x, start_point.y, end_point.x, end_point.y);
    				 // This is to avoid oscillation of the robot around the path
    				if(abs(cross_track_error) <= cross_track_error_deadband) {cross_track_error = 0.0;}

    				delta_time = ros::Time::now() - previous_time;
    				if(delta_time > ros::Duration(min_delta_time)) {
    					previous_time = ros::Time::now();
    					delta_cross_track_error = cross_track_error - previous_cross_track_error;
    					derivative_cross_track_error = derivative_cte_gain * delta_cross_track_error/delta_time.toSec();
    					previous_cross_track_error = cross_track_error;

    					delta_heading_error = heading_error - previous_heading_error;
    					derivative_heading_error = derivative_heading_gain * delta_heading_error/delta_time.toSec();
    					previous_heading_error = heading_error;
    				}

                    //==========================================================
                    // Set Steering Input
                    //==========================================================
                    if (control_mode == 1) {
                        // NOTE: because the forklift is rear-steering when going in the forward direction, a negative steering angle results in a left-hand turn which is a positive angular velocity. This controller was designed for a front-steering system, so the steering angle must be negated.
                        steering_angle = -(heading_gain*heading_error + cte_gain*cross_track_error + derivative_cross_track_error + derivative_heading_error);
                    }
                    else if (control_mode == 2) {
                        steering_angle = heading_gain*heading_error + cte_gain*cross_track_error + derivative_cross_track_error + derivative_heading_error;
                    }

                    // Bound the steering angle
                    steering_angle = max(steering_angle, steering_angle_min);
                    steering_angle = min(steering_angle, steering_angle_max);

                    //==========================================================
                    // Set Velocity Input
                    //==========================================================
                    linear_velocity = cos(steering_angle)*maximum_linear_velocity;

                    // Slowing down if approaching last goal
    				if (segment == local_path.size()-1) {
    					double segment_length = sqrt(pow((local_path[segment+1][0] - local_path[segment][0]),2) + pow((local_path[segment+1][1]- local_path[segment][1]),2));
    					double approaching_velocity = (abs(along_track_error)/segment_length)*linear_velocity;
    					linear_velocity = min(linear_velocity, approaching_velocity);
    				}

                    if (control_mode == 2) {
                        // if going in the reverse direction, negate the velocity
                        linear_velocity = -linear_velocity;
                    }

                    // Update the linear velocity to slow down if the current steering angle is far off from the desired setpoint
                    double current_angle_error = fabs(steering_angle - current_angle);
                    current_angle_error = min(current_angle_error, current_angle_error_max);
                    // Scale the angle error to a range of [0,pi/2]
                    current_angle_error *= (M_PI_2/current_angle_error_max);
                    // Scale the linear velocity by cos() of the error
                    linear_velocity *= cos(current_angle_error);

                    // DEBUG:
                    printf("[%s] heading: %0.04f, h: %0.04f, cte: %0.04f, der_cte: %0.04f, der_h: %0.04f, steer: %0.04f, vel: %0.04f\n", ros::this_node::getName().c_str(), forklift_heading, heading_gain*heading_error, cte_gain*cross_track_error, derivative_cross_track_error, derivative_heading_error, steering_angle, linear_velocity);

                    //==========================================================
                    // Publish Commands
                    //==========================================================
                    // (the logic is currently set up this way to make the intensions clear that no command should be sent when in "manual" mode)
                    if (manual_deadman_on and ((getWallTime() - timeout_start) < timeout)) {
                        // Send no command
                    }
                    else if (autonomous_deadman_on and ((getWallTime() - timeout_start) < timeout)) {
                        // Publish the raw Linear Velocity and Steering Angle
                        // NOTE: because the forklift is going in the reverse
                        // direction, the velocity must be made negative.
                        std_msgs::Float64 velocity_msg;
                        std_msgs::Float64 steer_msg;
                        velocity_msg.data = linear_velocity;
                        steer_msg.data = steering_angle;

                        lin_vel_pub.publish(velocity_msg);
                        steer_angle_pub.publish(steer_msg);
                    }
                    else {
                        // Joystick has timed out, send 0 velocity command
                        // Do not send a steering angle command, so it remains where it is currently at.
                        std_msgs::Float64 velocity_msg;
                        velocity_msg.data = 0.0;
                        lin_vel_pub.publish(velocity_msg);
                    }

                    // Publish lookahead path
    				vector<geometry_msgs::PoseStamped> plan;
    				ros::Time time_now = ros::Time::now();
    		        for (int i = segment; i < segment + lookahead_segments; i++) {
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
                }

                // Slow down the loop if the 'if' condition is being skipped because the current control mode turns this 'off'
                rate.sleep();

				along_track_error = compute_along_track_error();
			}

		    segment++;
		}

		return true;
	}

	double path_curvature(double present_heading, int lookahead_segments)
    {
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

	double lookAheadError(double present_heading, int lookahead_segments)
	{
		// Calculated the weighted error from considering the future steering angle
		double lookaheaderror = 0.0;
		double weight_sum = 0.0;
		for (int i = 1; i < lookahead_segments; ++i) {
			double heading = atan2(local_path[segment+i][1] - local_path[segment+(i-1)][1], local_path[segment+i][0] - local_path[segment+(i-1)][0]);
			double error = heading - present_heading;
			lookaheaderror += error/i;
			weight_sum += (1.0/i);
		}

        if (lookahead_segments == 1) {
            double heading = atan2(local_path[segment+1][1] - local_path[segment][1], local_path[segment+1][0] - local_path[segment][0]);
            double error = heading - present_heading;
            lookaheaderror = error;
            weight_sum = 1.0;
        }

		lookaheaderror /= weight_sum;

		return lookaheaderror;
	}

	void update_path(nav_msgs::Path path)
    {

        // // DEBUG: check if this function is running
        // cout << "*********************************************\n";
        // cout << "*** UPDATING PATH ***\n";
        // cout << "*********************************************" << endl;

		//pathIsAvailable = true;
		segment = 0;
		vector <double>  new_point;
		vector <vector <double> >  update_path;

		for (int i = 0; i < path.poses.size(); i++){
			new_point.push_back(path.poses[i].pose.position.x);
	        new_point.push_back(path.poses[i].pose.position.y);
	        update_path.push_back(new_point);
			new_point.clear();
		}
		local_path = update_path;
		update_path.clear();

		// if(path_tracking_controller()){
		// 	ROS_INFO("DONE!");
		// }

        // Once this is set to 'true', the control loop is able to run Starting
        // as 'false' gets this loop to run at least once before running the
        // control loop.
        path_has_been_updated = true;
	}

	double compute_cross_track_error(double x0, double y0, double x1, double y1, double x2, double y2)
    {
		double Dx, Dy, numerator, denominator, xte;
		Dx = x2 - x1;
		Dy = y2 - y1;
		numerator = Dy*x0 - Dx*y0 - x1*y2 + x2*y1;
		denominator = sqrt(Dx*Dx + Dy*Dy);
		xte = numerator/denominator;
		return xte;
	}

	double compute_along_track_error()
    {
		double L1, L2, L3;
	 	double theta, along_track_error;
        L1 = sqrt(pow((pose.x - local_path[segment+1][0]),2) + pow((pose.y - local_path[segment+1][1]),2));
        L2 = sqrt(pow((local_path[segment+1][0] - local_path[segment][0]),2) + pow((local_path[segment+1][1] - local_path[segment][1]),2));
        L3 = sqrt(pow((pose.x - local_path[segment][0]),2) + pow((pose.y - local_path[segment][1]),2));
        // theta = acos(-(L3*L3 - L1*L1 - L2*L2)/(2*L1*L2));
        // along_track_error = L1*cos(theta);

         along_track_error = -(L3*L3 - L1*L1 - L2*L2)/(2*L2);
		 return along_track_error;
	}

	bool aquire_robotpose()
    {
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
		pose.heading = wrapToPi(yaw);
		return true;
	}

	double smallest_diff_angle(double theta2, double theta1)
    {
		// create a quaternion for each angle
		// then multiply them with the other which is addition in quaternion
		// convert the result back to angle
		double result;
		tf::Quaternion quat2, quat1;
		quat1.setRPY(0.0, 0.0, theta1);
		quat2.setRPY(0.0, 0.0, theta2);
		return result=quat2.angleShortestPath(quat1);
	}

	void get_params()
    {
        // // Manually Set Parameters
        // nh_.setParam("maximum_linear_velocity", 0.5);
        // nh_.setParam("derivative_cte_gain", 0.1);
        // nh_.setParam("derivative_heading_gain", 0.01);
        // nh_.setParam("goal_tolerance",  0.1);

		//nh_.param("maximum_linear_velocity", maximum_linear_velocity, 1.0);
        nh_.param("maximum_linear_velocity", maximum_linear_velocity, 0.5);
		nh_.param("maximum_angular_velocity",maximum_angular_velocity, 1.92);
		nh_.param("num_of_segments_ahead", num_of_segments_ahead, 1);
		nh_.param("goal_tolerance", goal_tol, 0.3);
		nh_.param("steering_gain",steering_gain, 1.0);
	    nh_.param("error_gain",error_gain,1.0);
	    nh_.param("cte_gain", cte_gain, 0.01);
		nh_.param("heading_gain", heading_gain, 1.0);
		nh_.param("derivative_cte_gain",derivative_cte_gain, 1.0);
		nh_.param("derivative_heading_gain", derivative_heading_gain, 1.0);
		nh_.param("cross_track_error_deadband",cross_track_error_deadband, 0.3);
		nh_.param("min_delta_time",min_delta_time,0.1);

        nh_.param("manual_deadman", manual_deadman_button, 4);
        nh_.param("autonomous_deadman", autonomous_deadman_button, 5);
        nh_.param("timeout", timeout, 1.0);
        nh_.param("/forklift/steering/min_angle", steering_angle_min, -75*(M_PI/180.0));
        nh_.param("/forklift/steering/max_angle", steering_angle_max, 75*(M_PI/180.0));
        nh_.param("angle_error_max", current_angle_error_max, 75*(M_PI/180.0));
	}

	void parameter_callback(robust_navigation::GainsConfig &config, uint32_t level)
    {
		maximum_linear_velocity = config.maximum_linear_velocity;
		maximum_angular_velocity = config.maximum_angular_velocity;
		goal_tol = config.goal_tolerance;
		num_of_segments_ahead = config.num_of_segments_ahead;
		steering_gain = config.steering_gain;
	    error_gain = config.error_gain;
	    cte_gain = config.cte_gain;
		heading_gain = config.heading_gain;
		cross_track_error_deadband =  config.cross_track_error_deadband;
		derivative_cte_gain = config.derivative_cte_gain;
		derivative_heading_gain = config.derivative_heading_gain;
		min_delta_time = config.min_delta_time;
	}

    void joy_override(const sensor_msgs::Joy joy_msg)
    {
        // Update timeout time
        timeout_start = getWallTime();

        // Update deadman buttons
        if (joy_msg.buttons[manual_deadman_button] == 1) {
            manual_deadman_on = true;
        }
		else {
            manual_deadman_on = false;
        }

        if (joy_msg.buttons[autonomous_deadman_button] == 1) {
            autonomous_deadman_on = true;
        }
		else {
            autonomous_deadman_on = false;
        }
	}

    void gearCallback(const std_msgs::Int8 msg)
    {
        gear = msg.data;

        // Bound the gear to be -1, 0, or 1
        if (gear != 1 and gear != -1) {
            gear = 0;
        }
    }

    void angleCallback(const std_msgs::Float64 msg)
    {
        current_angle = msg.data;
    }

    void controlModeCallback(const std_msgs::Int8 msg)
    {
        control_mode = msg.data;
    }

    double wrapToPi(double angle)
    {
        angle = fmod(angle + M_PI, 2*M_PI);
        if (angle < 0) {
            angle += 2*M_PI;
        }
        return (angle - M_PI);
    }

    double getWallTime() {
        struct timeval time;
        if (gettimeofday(&time, NULL)) {
            return 0;
        }
        return (double)time.tv_sec + (double)time.tv_usec*0.000001;
    }

    bool checkControlMode(int mode, vector<int> vector_of_modes)
    {
        // Use 'find' on the vector to determine existence of 'mode'
        vector<int>::iterator it;
        it = find(vector_of_modes.begin(), vector_of_modes.end(), mode);
        if (it != vector_of_modes.end()) {
            return true;
        }
        else {
            return false;
        }
    }
};

int main(int argc, char **argv){

	ROS_INFO("initializing velocity_controller");
	ros::init(argc, argv, "velocity_controller");

	VelocityController VC;

	ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
	return 0;
}
