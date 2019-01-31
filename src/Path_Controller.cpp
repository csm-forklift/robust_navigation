// Obtain the global path from the global planner
// While goal has not been reached keep generating local plan for the path controller to track
// If there is a new global plan, update the local plan

#include <robust_navigation/Path_Controller.h>
#include <geometry_msgs/Twist.h>
//#include <robust_navigation/GetPlan.h>
#include <tf/tf.h>
#include <tf/LinearMath/Quaternion.h>





using namespace std;

namespace geobot_local_planner{

LocalPlannerUtil::LocalPlannerUtil(): nhPriv("~"), move2goal_action_(nh, "move2goal",boost::bind(&LocalPlannerUtil::run, this, _1), false), f(boost::bind(&LocalPlannerUtil::parameter_callback, this, _1, _2))
{
	//nhPriv.resolveName("~");
	//ros::NodeHandle nhPriv("~");		
	nhPriv.param("maximum_linear_velocity", maximum_linear_velocity, 1.0);
	nhPriv.param("maximum_angular_velocity", maximum_angular_velocity, 1.92);
	nhPriv.param("error_tolerance2goal", error_tolerance2goal, 0.4);
	nhPriv.param("no_of_segments_ahead", no_of_segments_ahead, 5);
	nhPriv.param("error_gain", error_gain,1.0);
	nhPriv.param("cte_gain", cte_gain,1.0);
	nhPriv.param("heading_gain", heading_gain,1.0);
	nhPriv.param("velocity_constraint", velocity_constraint,1.0);
	nhPriv.param("obstacle_edge2robotcenter", obstacle_clearance, 0.5); // should be <= half of gap_width threshold
	nhPriv.param("trajectory_projection", trajectory_projection, 3.0);
	nhPriv.param("cross_track_error_deadband", cross_track_error_deadband, 0.2);
	nhPriv.param("side_distance_avoidance_threshold", side_distance_avoidance_threshold, 0.8);
	nhPriv.param("front_distance_avoidance_threshold", front_distance_avoidance_threshold, 1.2);
	nhPriv.param("side_avoidance_blindband", side_avoidance_blindband, 0.4);
    nhPriv.param("derivative_gain", derivative_gain, 0.01);
	nhPriv.param("steering_gain", steering_gain, 1.0);
	nhPriv.param("min_delta_time", min_delta_time, 0.1);
	nhPriv.param("derivative_heading_gain",derivative_heading_gain, 0.1);

	// calling the parameter callback
	server.setCallback(f);

	//make parameter
	updatedpath_filename = "/home/blaster-ws/ros_ws/src/geobot_gui/src/real_path.txt";
	utmZone = "13S";



	gap_dist_right = 0.0;
	gap_dist_left = 0.0;
	gap_angle_right = 0.0;
	gap_angle_left = 0.0;
	path2select = ORIGINAL_PATH;
	//onwhich_sideofrobot_is_originalpath = LEFT_SIDE_OF_ROBOT;

	velocity_cmdPublisher_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 20); // publish the velocity commands
  	local_pathPublisher_ = nh.advertise<nav_msgs::Path>("local_path", 20); // publish the local_path
  	alternate_pathPublisher_ = nh.advertise<nav_msgs::Path>("alternate_path", 20); // publish the alternate_path
  	// poseSubscriber_ = nh.subscribe("item_order",1000, &LocalPlannerUtil::subscribe_for_present_pose,this); // subscribe for the present pose
  	pathSubscriber_ = nh.subscribe("path",10, &LocalPlannerUtil::update_path,this); 

  	//obstacle_proximitySubscriber_ = nh.subscribe("obstacle_proximity",10, &LocalPlannerUtil::acquire_obstacle_proximity,this); // subscribe for the present pose
  	//gap_arraySubscriber_ = nh.subscribe("candidate_gaps",10, &LocalPlannerUtil::acquire_candidate_gaps,this);
  	//joystickoverideSubscriber_ = nh.subscribe("/joy_teleop/joy",10, &LocalPlannerUtil::joystickoveride,this);


  	// Subscribe to the services we need
    //move2goal_ = nh.advertiseService("move2goal", &LocalPlannerUtil::run, this); // This is a move2goal server but action is now being used
    move2goal_action_.start();
    //makeplan_ = nh.serviceClient<robust_navigation::GetPlan>("makeplan"); 
    override_by_joystick = false;
    pathIsAvailable = false;
}


void LocalPlannerUtil::parameter_callback(robust_navigation::GainsConfig &config, uint32_t level)
{
	//Launch file variables that will be changed using
	//dynamic reconfigure!
	maximum_linear_velocity = config.maximum_linear_velocity;
	maximum_angular_velocity = config.maximum_angular_velocity;         
	error_tolerance2goal  = config.error_tolerance2goal;         
	no_of_segments_ahead = config.no_of_segments_ahead;       
	error_gain = config.error_gain;           
	cte_gain = config.cte_gain;             
	heading_gain = config.heading_gain;           
	velocity_constraint = config.velocity_constraint;        
	obstacle_clearance = config.obstacle_edge2robotcenter;    
	trajectory_projection = config.trajectory_projection;      
	cross_track_error_deadband = config.cross_track_error_deadband;   
	side_distance_avoidance_threshold = config.side_distance_avoidance_threshold;
	front_distance_avoidance_threshold = config.front_distance_avoidance_threshold;
	side_avoidance_blindband = config.side_avoidance_blindband;            

	 ROS_INFO("Junk!");        

}		

double LocalPlannerUtil::smallest_diff_angle(double theta2, double theta1)
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

double LocalPlannerUtil::path_curvature(double present_heading, int lookahead_segments)
{
	// compute the path curvature and add to the heading error between the robot heading and the path
	double lookahead_heading_deviation = 0.0;
	double init_angle = present_heading;
	double angle, diff_angle;

//	cout << "segment-----"<< segment << endl;
//	cout << "local_path size-----"<< local_path.size() << endl;

	for (int i = 1; i < lookahead_segments; i++)
	{
//		cout << "i-----"<< i << endl;

//		cout << "local_path[segment+i][1]-----"<< local_path[segment+i][1] << endl;
//		cout << "local_path[segment][1]-----"<< local_path[segment][1] << endl;
//		cout << "local_path[segment+i][0] -----"<< local_path[segment+i][0]  << endl;
//		cout << "local_path[segment][0]-----"<< local_path[segment][0] << endl;
		
		angle = atan2(local_path[segment+i][1] - local_path[segment][1], local_path[segment+i][0] - local_path[segment][0]);
//		cout << "angle-----"<< angle << endl;
		diff_angle = smallest_diff_angle(angle,  init_angle); // may require edit
//		cout << "diff_angle-----"<< diff_angle << endl;
		init_angle = angle;
		lookahead_heading_deviation = lookahead_heading_deviation + abs(diff_angle);
	}

	return lookahead_heading_deviation;

}



double LocalPlannerUtil::compute_cross_track_error(double x0, double y0, double x1, double y1, double x2, double y2) 
{
	// compute the cross track error to local path
	double Dx, Dy, numerator, denominator, xte;

	Dx = x2 - x1;
	Dy = y2 - y1;

	numerator = Dy*x0 - Dx*y0 - x1*y2 + x2*y1;
	denominator = sqrt(Dx*Dx + Dy*Dy);
	xte = numerator/denominator;


	return xte;

}



double LocalPlannerUtil::compute_along_track_error()
{

	 // compute the along track error on the local path
	 double L1 , L2, L3;
	 double theta, along_track_error;

	 L1 = sqrt(pow((pose.x - local_path[segment+1][0]),2) + pow((pose.y - local_path[segment+1][1]),2));
	 L2 = sqrt(pow((local_path[segment+1][0] - local_path[segment][0]),2) + pow((local_path[segment+1][1] - local_path[segment][1]),2));
	 L3 = sqrt(pow((pose.x - local_path[segment][0]),2) + pow((pose.y - local_path[segment][1]),2));

	 theta = acos(-(L3*L3 - L1*L1 - L2*L2)/(2*L1*L2));
	 along_track_error = L1*cos(theta);

	 return along_track_error;
}




bool LocalPlannerUtil::path_tracking_controller()
{
// Obtain present pose using lookup transform or subscribing for pose
// Track the local path
// publish the velocity command
// publish the local path within a window size

// Subscribe for obstacle configuration
// plan new route depending on obstacle configuration
// and track the new path until clear

// return 0 if goal unreachable

	double angular_velocity;
	double steering_angle;
	double along_track_error;
	int lookahead_segments, remaining_segments;
	double bendahead, linear_velocity;
	double goal_heading, error_heading, cross_track_error;
	double approaching_velocity, segment_length;
	int sequence = 0;
	bool obstacle_on_the_way = false;
	double previous_cross_track_error = 0.0;
	ros::Time previous_time = ros::Time::now();
	ros::Duration delta_time;
	double delta_cross_track_error = 0.0;
	double derivative_error_term = 0.0;
	double derivative_heading_error_term = 0.0;
	double previous_error_heading = 0.0;
	double delta_error_heading = 0.0;

	geometry_msgs::Twist velocity_command;
	vector<geometry_msgs::PoseStamped> plan;
	nav_msgs::Path localplan2publish, alternateplan2publish;
	
	Path_definition start_point, end_point;
	double LR, LL, L3, LR_heading, LL_heading, x1, x2, y1, y2, x_t, offset_direction, path_offset, x_center, y_center, new_traj_heading;
	double LR_x, LR_y, LL_x, LL_y;

///////////////////////////////////////////////////////////////////////////////////
	  //Listen for the transorm from odom_utm so that paths can be displayed on the map for visualization
	  tf::StampedTransform utm_odom_transform;
	  tf::Transform odom_Path;
	  tf::Transform transform2utm;
	  bool obtained_transform2utm = false;
	  string data_string;
	  double gpr_x;
	  double gpr_y;
	  double gpr_latitude;
	  double gpr_longitude;

	  try
	  {
	    listener.waitForTransform("utm","odom", ros::Time(0), ros::Duration(1.0));
	    listener.lookupTransform("utm", "odom", ros::Time(0), utm_odom_transform); 
	    obtained_transform2utm = true; 
	  }
	  catch(tf::TransformException &ex)
	  {
	     ROS_ERROR("%s",ex.what());
	     
	  }


	  //////////////////////////////////////


	// Try tracking the given local path
	while(segment < local_path.size()-1)
	{
		
//		cout << "segment counter-----"<< segment << endl;

		 if(obtained_transform2utm)
         {
            // convert to longitude and latitude
            odom_Path.setOrigin(tf::Vector3(local_path[segment][0], local_path[segment][1], 0.0));  // Easting, Northing, Altitude
            transform2utm.mult(utm_odom_transform, odom_Path);
            gpr_x = transform2utm.getOrigin().getX();
            gpr_y = transform2utm.getOrigin().getY();
            RobotLocalization::NavsatConversions::UTMtoLL(gpr_y, gpr_x, utmZone, gpr_latitude, gpr_longitude);

      //      fileOUT << gpr_longitude << "\t" << gpr_latitude << endl;
         }

		
		along_track_error = compute_along_track_error();
//		cout << "along_track_error_start-----"<< along_track_error << endl;

		while (along_track_error > error_tolerance2goal) 
		{
			// Obtain present pose using lookup transform
			acquire_robotpose(); // pose is a private variable

			
			// If there is no obstacle, track the original local path
			////////////////////////////////////////////////////////////////////////////////////////////////////////
			obstacle_on_the_way = false;
			start_point.x = local_path[segment][0] ;
			start_point.y = local_path[segment][1] ;
			end_point.x = local_path[segment+1][0] ;
			end_point.y = local_path[segment+1][1] ;



				remaining_segments = local_path.size() - segment;
//				cout << "remaining_segments-----"<< remaining_segments << endl;
				lookahead_segments = min(no_of_segments_ahead, remaining_segments); 
//				cout << "lookahead_segments-----"<< lookahead_segments << endl;
				bendahead = path_curvature(pose.heading, lookahead_segments); 
//				cout << "bendahead-----"<< bendahead << " pose.heading-----"<< pose.heading << " pose.x-----"<< pose.x << " pose.y-----"<< pose.y << endl;
			// }



			// Adjusting velocity with road or path curvature
			// In case of rough terrain, this will be the place to adjust the velocity	
			velocity_constraint = (1 - pow((bendahead/(3.14*0.6)),1.5));
//			cout << "velocity_constraint-----"<< velocity_constraint << endl;
			velocity_constraint = max(velocity_constraint,0.3);
			linear_velocity = velocity_constraint*maximum_linear_velocity; 


			// Slowing down proportionally to the end of final goal
			if (segment == local_path.size()-1)
			{
				segment_length = sqrt(pow((local_path[segment+1][0] - local_path[segment][0]),2) + pow((local_path[segment+1][1]- local_path[segment][1]),2));	
				approaching_velocity = (abs(along_track_error)/segment_length)*linear_velocity;
				linear_velocity = min(linear_velocity, approaching_velocity);
			}
//			cout << "linear_velocity-----"<< linear_velocity << endl;

			// compute the required angular velocity
			goal_heading = atan2(end_point.y - start_point.y, end_point.x - start_point.x);
//			cout << "goal_heading-----"<< goal_heading << endl;
			//error_heading = smallest_diff_angle(goal_heading, pose.heading); // check this

			error_heading = goal_heading - pose.heading;
			// computing smallest angle
			if (error_heading > M_PI)
			{
				error_heading = error_heading - (M_PI*2);
			}

			if (error_heading < -M_PI)
			{
				error_heading = error_heading + (M_PI*2);
			}



//			cout << "error_heading-----"<< error_heading << endl;

			cross_track_error = compute_cross_track_error(pose.x, pose.y, start_point.x, start_point.y, end_point.x, end_point.y);

			if(abs(cross_track_error) <= cross_track_error_deadband)  // This is to avoid oscillation of the robot around the path
			{
				cross_track_error = 0.0;
			}

//			cout << "cross_track_error-----"<< cross_track_error << endl;
			delta_time = ros::Time::now() - previous_time;
			
			
             		if(delta_time > ros::Duration(min_delta_time))
			{
			
				previous_time = ros::Time::now();
				delta_cross_track_error = cross_track_error - previous_cross_track_error;
				derivative_error_term = derivative_gain * delta_cross_track_error/delta_time.toSec();
				previous_cross_track_error = cross_track_error;

				delta_error_heading = error_heading - previous_error_heading;
				derivative_heading_error_term = derivative_heading_gain * delta_error_heading/delta_time.toSec();
				previous_error_heading = error_heading;
			}
			
			ROS_INFO("derivative component  %f", derivative_error_term);
			ROS_INFO("delta time component  %f", delta_time.toSec());
			ROS_INFO("error difference component  %f", delta_cross_track_error);



			steering_angle = heading_gain*error_heading + error_gain*atan2(cte_gain*cross_track_error,linear_velocity) + derivative_error_term + derivative_heading_error_term;  //check this
			
			

			angular_velocity = steering_gain * steering_angle;

//			cout << "angular_velocity-----"<< angular_velocity << endl;

			if (angular_velocity > maximum_angular_velocity)
			{angular_velocity = maximum_angular_velocity;}

			if (angular_velocity < -maximum_angular_velocity)
				{angular_velocity = -maximum_angular_velocity;}

			//if there is no gap, that is,  if trapped
			// then slowly turn in place
			/*
			if(gap_position2obstacle == NO_GAP && !override_by_joystick)
			{
				linear_velocity = 0.0;
				angular_velocity = -0.4;
			}
			*/

 
			// then publish the commanded velocity
			velocity_command.linear.x = linear_velocity;
			velocity_command.linear.y= 0.0;
			velocity_command.linear.z= 0.0;

			velocity_command.angular.x = 0.0;
			velocity_command.angular.y = 0.0;
			velocity_command.angular.z = angular_velocity;

			velocity_cmdPublisher_.publish(velocity_command);


			

			// Publish out the local plan presently followed
			ros::Time time_now = ros::Time::now();

	        for (int i = segment; i < segment + lookahead_segments; i++)
	        {
	          geometry_msgs::PoseStamped local_plan;
	          tf::Quaternion quat = tf::createQuaternionFromYaw(0);

	          local_plan.header.seq = i;
	          local_plan.header.stamp = time_now;
	          local_plan.header.frame_id = "odom";
          
	          local_plan.pose.position.x = local_path[i][0];  // Latitude
	          local_plan.pose.position.y = local_path[i][1];  // Longitude

	          // cout << "x------"<< local_path[i][0] << "y------" << local_path[i][1] << endl;
	     
	          local_plan.pose.orientation.x = quat.x();
	          local_plan.pose.orientation.y = quat.y();
	          local_plan.pose.orientation.z = quat.z();
	          local_plan.pose.orientation.w = quat.w();
	     
	          plan.push_back(local_plan);
	      	}


	      	localplan2publish.poses = plan;
		    localplan2publish.header.seq = sequence++;
		    localplan2publish.header.stamp = ros::Time::now();
		    localplan2publish.header.frame_id = "odom";
	      	local_pathPublisher_.publish(localplan2publish);
	      	plan.clear();


	      	// Publish the alternate path to avoid the obstacle
		    //if(obstacle_on_the_way)
		    //{
		      geometry_msgs::PoseStamped alternate_plan;
	          tf::Quaternion alternate_quat = tf::createQuaternionFromYaw(0);

	          alternate_plan.header.seq = 0;
	          alternate_plan.header.stamp = time_now;
	          alternate_plan.header.frame_id = "odom";
          
	          alternate_plan.pose.position.x = start_point.x;  // Latitude
	          alternate_plan.pose.position.y = start_point.y;  // Longitude
	         	     
	          alternate_plan.pose.orientation.x = alternate_quat.x();
	          alternate_plan.pose.orientation.y = alternate_quat.y();
	          alternate_plan.pose.orientation.z = alternate_quat.z();
	          alternate_plan.pose.orientation.w = alternate_quat.w();
	          plan.push_back(alternate_plan);

	          alternate_plan.header.seq = 1;
	          alternate_plan.header.stamp = time_now;
	          alternate_plan.header.frame_id = "odom";
          
	          alternate_plan.pose.position.x = end_point.x;  // Latitude
	          alternate_plan.pose.position.y = end_point.y;  // Longitude
	         	     
	          alternate_plan.pose.orientation.x = alternate_quat.x();
	          alternate_plan.pose.orientation.y = alternate_quat.y();
	          alternate_plan.pose.orientation.z = alternate_quat.z();
	          alternate_plan.pose.orientation.w = alternate_quat.w();
	          plan.push_back(alternate_plan);

  	      	  alternateplan2publish.poses = plan;
			  alternateplan2publish.header.seq = localplan2publish.header.seq;
			  alternateplan2publish.header.stamp = ros::Time::now();
			  alternateplan2publish.header.frame_id = "odom";
		      alternate_pathPublisher_.publish(alternateplan2publish);
		      plan.clear();

		    //}


			along_track_error = compute_along_track_error(); 
//			cout << "along_track_error-----"<< along_track_error << endl;

		}


		segment++;
	}

	//fileOUT.close();

	return true;

}

bool LocalPlannerUtil::acquire_robotpose()
{
	tf::StampedTransform odom_base_transform;
	tf::Matrix3x3 mat;
	double roll;
	double pitch;
	double yaw;

	
	try{
	listener.lookupTransform("odom","base_link", ros::Time(0), odom_base_transform);	
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



// Caveat: local_path variable cannot be overwritten while tracking is being performed ---Why?

void LocalPlannerUtil::update_path(nav_msgs::Path path)
{
	// fill in the path message into a vector local path	
	pathIsAvailable = true;
	segment = 0;
	vector <double>  new_point; 
	vector <vector <double> >  update_path;
	for (int i = 0; i < path.poses.size(); i++)
	{
		new_point.push_back(path.poses[i].pose.position.x);
        new_point.push_back(path.poses[i].pose.position.y);
        update_path.push_back(new_point);
		new_point.clear();
	}

	local_path = update_path;
	update_path.clear();


}


// This is to drive to a goal
//bool LocalPlannerUtil::run(geobot_navigation::SetGoal::Request& req, geobot_navigation::SetGoal::Response& resp)
bool LocalPlannerUtil::run(const robust_navigation::SetGoalGoalConstPtr& req)
{

	
// Acquire robot pose
// Check if path is available
// Assign the path to the local path
// Call the path_tracking_controller
// Alert the operator of goal completion


	ROS_INFO("Executing Path");

	nhPriv.getParam("maximum_linear_velocity", maximum_linear_velocity);
	nhPriv.getParam("maximum_angular_velocity", maximum_angular_velocity);
	nhPriv.getParam("error_tolerance2goal", error_tolerance2goal);
	nhPriv.getParam("no_of_segments_ahead", no_of_segments_ahead);
	nhPriv.getParam("error_gain", error_gain);
	nhPriv.getParam("cte_gain", cte_gain);
	nhPriv.getParam("heading_gain", heading_gain);
	nhPriv.getParam("velocity_constraint", velocity_constraint);

	bool goal_reached = false;


	if(!acquire_robotpose()) // update the robot pose
	{
		ROS_ERROR("An error as occurred acquiring robot pose");
		return goal_reached;
	}



	
		

	// check if path is available

	if (pathIsAvailable)
	{
		segment = 0;
		goal_reached = path_tracking_controller();
		local_path.clear();
		pathIsAvailable = false;
	}
	else
		{ROS_INFO("Global path is not acquired");}

	if(goal_reached)
	{
		ROS_INFO("Goal is reached");
	}
	else
	{
		ROS_INFO("Goal unreachable");
	}


	return goal_reached;

}


//callback for jostick override over false obstacle
void LocalPlannerUtil::joystickoveride(const sensor_msgs::Joy joy_msg)
{
    if(joy_msg.buttons[6] == 1) // so that debugging can be done if the other trigger is pressed
    {
        override_by_joystick = true;
    }
    else
    {
        override_by_joystick = false;
    }

}



}





int main(int argc, char **argv)
{
  // initializing ros
  ROS_INFO(" initializing path_controller_node");
  ros::init(argc, argv, "path_controller_node");

  geobot_local_planner::LocalPlannerUtil LPU;

  //ros::spin();

  ros::MultiThreadedSpinner spinner(4);
  spinner.spin();

}
