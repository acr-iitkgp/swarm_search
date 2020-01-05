#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h> 
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/CommandTOL.h>
#include <ros/ros.h>
#include <ros/duration.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include "std_msgs/Float64.h"
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <unistd.h>
#include <time.h>
#include <cmath>
#include <math.h>

#include <geographic_msgs/GeoPoseStamped.h>
// #include <swarm_search/sip_goal.h>
#include <swarm_search/sip_goal2.h>
#include <swarm_search/point_list.h>
#include <swarm_search/local_flags.h>

using namespace std;

//Set global variablesros

double takeoff_alt = 1.5;
double search_altitude = 5.0;
double scan_altitude = 7.0;
double goal_tolerance = 0.003;

int cnt =0;

bool connected;
bool armed = 0;
bool reached_target;
int detected_points = 0;

bool takeoff_drone; // to tell when to takeoff 
bool land_drone; // to tell when to land

bool takeoff_done = 0;
bool land_done;
bool ROI_scan_flag = 0;

double delta_to_destination;
double prev_delta = 0.0;

int time_tolerance_to_destination = 100000;

mavros_msgs::State current_state;
sensor_msgs::NavSatFix current_pose; //current drone pose
geographic_msgs::GeoPoseStamped pose; //target pose - updated in function setDestination
geometry_msgs::Quaternion current_orientation, average_orientation;

swarm_search::point_list arr;
swarm_search::sip_goal2 master_goal2;
// swarm_search::sip_goal master_goal;
swarm_search::local_flags flags;

// for VELOCITY RECOVERY 
geometry_msgs::PoseStamped local_current_pose;
geometry_msgs::Pose2D local_target;
geometry_msgs::Pose2D g2l;
mavros_msgs::PositionTarget vel_msg;
geometry_msgs::Twist vel_msg_twist;

/*
flags.scan_flag.data 
flags.transition_s2s.data - transition scan to search
flags.search_flag.data 
flags.recovery1_flag.data 
flags.recovery2_flag.data 
flags.wait.data // drone busy
*/

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
	current_state = *msg;
	bool connected = current_state.connected;
	bool armed = current_state.armed;
}

//get current position of drone
int counter_1 = 0;
float initial_takeoff_height = 0;

void pose_cb(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
	current_pose = *msg;
	if(counter_1<10)
	{
		initial_takeoff_height += current_pose.altitude;
		counter_1 += 1;
	}
	else if( counter_1 == 10)
	{
	initial_takeoff_height = initial_takeoff_height/10;
	counter_1+=1;
	}
	// ROS_INFO("Latitude %f Longitude: %f Altitude: %f", current_pose.latitude, current_pose.longitude, current_pose.altitude);
}

int counter_2 = 0;

void orientation_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	if(counter_2<30)
	{
		current_orientation = msg->pose.orientation; 
		average_orientation.w += current_orientation.w;
		average_orientation.x += current_orientation.x;
		average_orientation.y += current_orientation.y;
		average_orientation.z += current_orientation.z;

		counter_2+= 1;
	}
	else if( counter_2 == 30)
	{
		current_orientation.w = average_orientation.w/30;
		current_orientation.x = average_orientation.x/30;
		current_orientation.y = average_orientation.y/30;
		current_orientation.z = average_orientation.z/30;
		counter_2+=1;
	
	}
}

// VELOCITY RECOVERY ********************
void local_targetpose_cb(const geometry_msgs::Pose2D::ConstPtr& msg)
{
	local_target = *msg;
	//ROS_INFO("LOCAL x: %f y: %f ", local_target.x, local_target.y);
}

void local_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	local_current_pose = *msg;
	//ROS_INFO("LOCAL x: %f y: %f ", local_current_pose.pose.position.x, local_current_pose.pose.position.y);
}
// VELOCITY RECOVERY ********************

void callback_sip2(const swarm_search::sip_goal2::ConstPtr& msg)
{
	master_goal2 = *msg;
	//
}


double haversine(double lat1, double lon1, double lat2, double lon2) 
{ 
		// distance between latitudes 
		// and longitudes 
		double dLat = (lat2 - lat1) * M_PI / 180.0; 
		double dLon = (lon2 - lon1) * M_PI / 180.0; 

		// convert to radians 
		lat1 = (lat1) * M_PI / 180.0; 
		lat2 = (lat2) * M_PI / 180.0; 

		// apply formulae 
		double a = pow(sin(dLat / 2), 2) + pow(sin(dLon / 2), 2)*cos(lat1)*cos(lat2); 
		double rad = 6375; 
		double c = 2 * asin(sqrt(a)); 
		return rad * c; 
} 

// VELOCITY RECOVERY ********************
int velocity_recovery(ros::NodeHandle nh, geographic_msgs::GeoPoseStamped pose1)
{
	ros::Rate rate(10.0);
	// ros::Publisher pub2 = nh.advertise<mavros_msgs::PositionTarget>("/drone1/mavros/setpoint_raw/local",50);
	ros::Publisher pub1 = nh.advertise<geometry_msgs::Twist>("/drone1/mavros/setpoint_velocity/cmd_vel_unstamped",20);
	ROS_INFO(" VELOCITY RECOVERY MODE CALLED ! ");

	int recovery_timeout = 300;
	int recovery_tolerance_time = 0;
	float vel_factor = 1;
	double dir_vec_mag = 0;
	double dir_vec_x = 0;
	double dir_vec_y = 0;
	
	double target_x = local_target.x;
	double target_y = local_target.y;
	double curr_x = 0;
	double curr_y = 0;

	// if(ros::ok())
	// {
	//   cout<<" ROS OK ! "<<ros::ok()<<endl;
	// } 
	// if(delta_to_destination > goal_tolerance)
	// {
	//   cout<<" delta_to_destination TRUE "<<endl;
	// }

	while( ros::ok() && (delta_to_destination > goal_tolerance) && (recovery_tolerance_time < recovery_timeout))
	{
		//ROS_INFO(" WHILE LOOP ME GHUSSA ! ");
		delta_to_destination = haversine(pose1.pose.position.latitude,pose1.pose.position.longitude,current_pose.latitude,current_pose.longitude);
		recovery_tolerance_time ++;

		curr_x = local_current_pose.pose.position.x;
		curr_y = local_current_pose.pose.position.y;

		dir_vec_mag = sqrt((curr_x-target_x)*(curr_x-target_x) + (curr_y-target_y)*(curr_y-target_y)); // altitute ke liye : (curr.z-target.z)*(curr.z-target.z))
		dir_vec_x = (target_x - curr_x)/(dir_vec_mag + 0.00001);
		dir_vec_y = (target_y - curr_y)/(dir_vec_mag + 0.00001);

		// vel_msg.coordinate_frame=mavros_msgs::PositionTarget::FRAME_BODY_NED;
		// vel_msg.header.frame_id="target_position";   

		// vel_msg.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
		// 										mavros_msgs::PositionTarget::IGNORE_PY |
		// 										mavros_msgs::PositionTarget::IGNORE_PZ |
		// 										mavros_msgs::PositionTarget::IGNORE_AFX |
		// 										mavros_msgs::PositionTarget::IGNORE_AFY |
		// 										mavros_msgs::PositionTarget::IGNORE_AFZ |
		// 										mavros_msgs::PositionTarget::FORCE |
		// 										mavros_msgs::PositionTarget::IGNORE_YAW |
		// 										mavros_msgs::PositionTarget::IGNORE_YAW_RATE ;

		// vel_msg.header.stamp = ros::Time::now();

		// vel_msg.velocity.x= dir_vec_x*vel_factor;//v.linear.x;  // *** DIRECTION KA DOUBT HAI -ve try karke dekho
		// vel_msg.velocity.y= dir_vec_y*vel_factor;//v.linear.y;
		// vel_msg.velocity.z= 0;
		
		vel_msg_twist.linear.x = dir_vec_y*vel_factor; 
		vel_msg_twist.linear.y = dir_vec_x*vel_factor;
		vel_msg_twist.linear.z = 0;

		pub1.publish(vel_msg_twist);
		ROS_INFO("DELTA_TO_DEST: %f  VELOCITY:: X: %f Y: %f ",delta_to_destination, vel_msg_twist.linear.x, vel_msg_twist.linear.y);
		//pub2.publish(vel_msg);
			
		ros::spinOnce();
	}
	
	return 1;  
}
// VELOCITY RECOVERY ********************


int navigate(ros::NodeHandle nh, geographic_msgs::GeoPoseStamped pose1)
{
	// the setpoint publishing rate MUST be faster than 2Hz
	ros::Rate rate(10.0);

	// cout << pose1 << endl;

	ros::Publisher global_pos_pub = nh.advertise<geographic_msgs::GeoPoseStamped>("/drone1/mavros/setpoint_position/global_to_local", 10);
	ros::Publisher cvt_g2l = nh.advertise<geometry_msgs::Pose2D>("/drone1/convert_global_to_local", 10); // change topic
	// ros::Publisher pub3 = nh.advertise<geometry_msgs::Twist>("/drone1/mavros/setpoint_velocity/cmd_vel_unstamped",20);
 	ros::Publisher pub1 = nh.advertise<geometry_msgs::Twist>("/drone1/mavros/setpoint_velocity/cmd_vel_unstamped",20);


	//DRONE DIAG FLAGS TO BE ADDED 

	// allow the subscribers to initialize
	ROS_INFO("INITIALISING...");
	for(int i=0; i<50; i++)
	{
		ros::spinOnce();
		ros::Duration(0.01).sleep();
	}

	pose1.header.stamp = ros::Time::now();
	global_pos_pub.publish(pose1);

	int tolerance_time = 0;
	int x;

	 //////////////////////////////////////// Parameters
	double target_x = 0;
	double target_y = 0;
	double curr_x = 0;
	double curr_y = 0;

	int recovery_timeout = 200;
	int recovery_tolerance_time = 0;
	float vel_factor = 0.7;
	double dir_vec_mag = 0;
	double dir_vec_x = 0;
	double dir_vec_y = 0;
	int flag_vel = 0;
	///////////////////////////////////
	if(global_pos_pub)
	{  //to reach the destination, it returns 1 when it has reached the destination
	
		while(ros::ok())
		{
			delta_to_destination = haversine(pose1.pose.position.latitude,pose1.pose.position.longitude,current_pose.latitude,current_pose.longitude);

			ROS_INFO("Delta to Destination: %f  Delta to Altitude: %f ", delta_to_destination, abs(pose1.pose.position.altitude-current_pose.altitude));

			pose1.header.stamp = ros::Time::now();
			if(flag_vel == 0) global_pos_pub.publish(pose1);

			g2l.x = pose1.pose.position.latitude;
			g2l.y = pose1.pose.position.longitude;
			cvt_g2l.publish(g2l);


			if(abs(delta_to_destination - prev_delta) < 0.00005 && abs(pose1.pose.position.altitude-current_pose.altitude) < 0.7)
			{
				tolerance_time++;
				ROS_INFO("tolerance time is : %d", tolerance_time);
			}

			if((tolerance_time > 50) && (tolerance_time < 250) && (delta_to_destination > goal_tolerance))
			// if(delta_to_destination < 0.005 &&delta_to_destination > goal_tolerance)
			{
				//x =	velocity_recovery(nh,pose1);
				target_x = local_target.x;
		        cout<<local_target.x<<" "<<local_target.y<<endl;
		        target_y = local_target.y;
		        curr_x = local_current_pose.pose.position.x;
		        curr_y = local_current_pose.pose.position.y;

		        dir_vec_mag = sqrt((curr_x-target_x)*(curr_x-target_x) + (curr_y-target_y)*(curr_y-target_y)); // altitute ke liye : (curr.z-target.z)*(curr.z-target.z))
		        dir_vec_x = (target_x - curr_x)/(dir_vec_mag + 0.00001);
		        dir_vec_y = (target_y - curr_y)/(dir_vec_mag + 0.00001);

		        vel_msg_twist.linear.x = dir_vec_x*vel_factor; 
		        vel_msg_twist.linear.y = dir_vec_y*vel_factor;
		        vel_msg_twist.linear.z = 0;
		        tolerance_time++;
		        pub1.publish(vel_msg_twist);
		        flag_vel = 1;
			}
			else
				flag_vel = 0;
			// if(delta_to_destination <0.006 && delta_to_destination > goal_tolerance)
			// {
			// 	x = velocity_recovery(nh,pose1);
			// }

			if( ((delta_to_destination < goal_tolerance) && abs(pose1.pose.position.altitude-current_pose.altitude)< 0.2) || tolerance_time > 250) /////////////////////change this//////////
			{
				ROS_INFO("Reached at the target position");  
				ROS_INFO("tolerance time is : %d", tolerance_time);
				flag_vel = 0;
				return 1;
			}

			prev_delta = delta_to_destination;

			ros::spinOnce();
			rate.sleep();
		}
	}
}


bool arm_drone(ros::NodeHandle nh)
{
	// arming
	ros::ServiceClient arming_client_i = nh.serviceClient<mavros_msgs::CommandBool>("/drone1/mavros/cmd/arming");
	mavros_msgs::CommandBool srv_arm_i;
	srv_arm_i.request.value = true;
	if (arming_client_i.call(srv_arm_i) && srv_arm_i.response.success){
		ROS_INFO("ARM sent %d", srv_arm_i.response.success);
		cout << "*******************************************************************" << endl;
		return 1;
	}
	else
	{
		ROS_ERROR("Failed arming");
		return 0;
	}
}

bool takeoff(ros::NodeHandle nh, double takeoff_alt_local)
{
		//request takeoff
		ros::ServiceClient takeoff_cl = nh.serviceClient<mavros_msgs::CommandTOL>("/drone1/mavros/cmd/takeoff");
		mavros_msgs::CommandTOL srv_takeoff;
		srv_takeoff.request.altitude = takeoff_alt_local;
		if(takeoff_cl.call(srv_takeoff)){
			ROS_INFO("takeoff sent %d", srv_takeoff.response.success);
			sleep(5);
			return 1;
		}
		else{
			ROS_ERROR("Failed Takeoff");
			return 0;
		}
		sleep(5);
}

bool land(ros::NodeHandle nh)
{
	ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("/drone1/mavros/cmd/land");
	mavros_msgs::CommandTOL srv_land;
	if (land_client.call(srv_land) && srv_land.response.success)
	{
		ROS_INFO("land sent %d", srv_land.response.success);
		sleep(10);
		return 1;
	}
	else
	{
		ROS_ERROR("Landing failed");
		ros::shutdown();                 /////////////////////////////////////////////////doubt here
		return 0;
	}
}


int check_ROI_confidence()
{
	// to be done
	return  1;
}


void roi_list(const swarm_search::point_list::ConstPtr& msg)
{
	arr = *msg;
}

void scan_main(ros::NodeHandle nh)
{

	ros::Publisher flags_pub = nh.advertise<swarm_search::local_flags>("/drone1/flags", 10);
	
	pose.header.stamp = ros::Time::now();
	pose.header.frame_id = "target_position";
	pose.pose.position.latitude = master_goal2.sip1.x;
	pose.pose.position.longitude = master_goal2.sip1.y;
	pose.pose.position.altitude = initial_takeoff_height + scan_altitude;
	pose.pose.orientation = current_orientation;

	int x = navigate(nh, pose); // returns 1 when reached
	
	if(x == 1)
	{
		ROS_INFO(" SIP 1 Reached ");
		ROS_INFO(" GOING TO SEARCH ");
	}
	
	pose.header.stamp = ros::Time::now();
	pose.header.frame_id = "target_position";
	pose.pose.position.latitude = master_goal2.sip1.x;
	pose.pose.position.longitude = master_goal2.sip1.y;
	pose.pose.position.altitude = initial_takeoff_height + search_altitude;
	pose.pose.orientation = current_orientation;

	x = 0;
	x = navigate(nh, pose); // returns 1 when reached
	
	if(x == 1)
	{
		ROS_INFO(" SIP 1 Reached - SEARCH HEIGHT");
		ROS_INFO(" Requesting ROI Scanning to start ");

		flags.search_flag.data = 1; 
		flags.scan_flag.data = 1; 
		for (int j= 0; j < 5; ++j)
		{
			flags_pub.publish(flags);
			sleep(0.1);
		} 
		//scanning should start here
		sleep(2);
	}

	pose.header.stamp = ros::Time::now();
	pose.header.frame_id = "target_position";
	pose.pose.position.latitude = master_goal2.sip2.x;
	pose.pose.position.longitude = master_goal2.sip2.y;
	pose.pose.position.altitude = initial_takeoff_height + search_altitude;
	pose.pose.orientation = current_orientation;

	////////////////////
	x = 0;
	x = navigate(nh, pose); // returns 1 when reached

	if(x == 1)
	{
		ROS_INFO(" SIP 2 Reached ");
		ROS_INFO(" Requesting ROI Scanning to continue ");

		flags.search_flag.data = 1; 
		for (int j= 0; j < 5; ++j)
		{
			flags_pub.publish(flags);
			sleep(0.1);
		} 
	sleep(2);
	}

	pose.header.stamp = ros::Time::now();
	pose.header.frame_id = "target_position";
	pose.pose.position.latitude = master_goal2.sip3.x;
	pose.pose.position.longitude = master_goal2.sip3.y;
	pose.pose.position.altitude = initial_takeoff_height + search_altitude;
	pose.pose.orientation = current_orientation;

	////////////////////
	x = 0;
	x = navigate(nh, pose); // returns 1 when reached

	if(x == 1)
	{
		ROS_INFO(" SIP 3 Reached ");
		ROS_INFO(" Requesting ROI Scanning to continue ");

		flags.search_flag.data = 1; 
		for (int j= 0; j < 5; ++j)
		{
			flags_pub.publish(flags);
			sleep(0.1);
		} 
	sleep(2);
	}

	pose.header.stamp = ros::Time::now();
	pose.header.frame_id = "target_position";
	pose.pose.position.latitude = master_goal2.sip4.x;
	pose.pose.position.longitude = master_goal2.sip4.y;
	pose.pose.position.altitude = initial_takeoff_height + search_altitude;
	pose.pose.orientation = current_orientation;

	////////////////////
	x = 0;
	x = navigate(nh, pose); // returns 1 when reached

	if(x == 1)
	{
		ROS_INFO(" SIP 4 Reached ");
		ROS_INFO(" Requesting ROI Scanning to continue ");

		flags.search_flag.data = 1; 
		for (int j= 0; j < 5; ++j)
		{
			flags_pub.publish(flags);
			sleep(0.1);
		} 

	sleep(2);
	}

	pose.header.stamp = ros::Time::now();
	pose.header.frame_id = "target_position";
	pose.pose.position.latitude = master_goal2.sip5.x;
	pose.pose.position.longitude = master_goal2.sip5.y;
	pose.pose.position.altitude = initial_takeoff_height + search_altitude;
	pose.pose.orientation = current_orientation;

	////////////////////
	x = 0;
	x = navigate(nh, pose); // returns 1 when reached

	if(x == 1)
	{
		ROS_INFO(" SIP 5 Reached ");
		ROS_INFO(" Requesting ROI Scanning to continue ");

		flags.search_flag.data = 1; 
		for (int j= 0; j < 5; ++j)
		{
			flags_pub.publish(flags);
			sleep(0.1);
		} 
	sleep(2);
	}

	pose.header.stamp = ros::Time::now();
	pose.header.frame_id = "target_position";
	pose.pose.position.latitude = master_goal2.sip6.x;
	pose.pose.position.longitude = master_goal2.sip6.y;
	pose.pose.position.altitude = initial_takeoff_height + search_altitude;
	pose.pose.orientation = current_orientation;

	////////////////////
	x = 0;
	x = navigate(nh, pose); // returns 1 when reached

	if(x == 1)
	{
		ROS_INFO(" SIP 6 Reached ");
		ROS_INFO(" SCAN AND SEARCH COMPLETE... LANDING! ");

		flags.search_flag.data = 1; 
		for (int j= 0; j < 5; ++j)
		{
			flags_pub.publish(flags);
			sleep(0.1);
		} 
	sleep(2);
	}

	flags.search_flag.data = 0; 
	for (int j= 0; j < 5; ++j)
	{
		flags_pub.publish(flags);
		sleep(0.1);
	} 
	bool z = land(nh);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "drone1_master_node");
	ros::NodeHandle nh;
	// the setpoint publishing rate MUST be faster than 2Hz
	ros::Rate rate(20.0);

	// mavros topics
	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/drone1/mavros/state", 10, state_cb);
	ros::Publisher global_pos_pub = nh.advertise<geographic_msgs::GeoPoseStamped>("/drone1/mavros/setpoint_position/global_to_local", 10);
	ros::Subscriber localPos = nh.subscribe<geometry_msgs::PoseStamped>("/drone1/mavros/local_position/pose", 10, orientation_cb);
	ros::Subscriber currentPos = nh.subscribe<sensor_msgs::NavSatFix>("/drone1/mavros/global_position/global", 10, pose_cb);
	ros::Subscriber local_currentPos1 = nh.subscribe<geometry_msgs::PoseStamped>("/drone1/mavros/local_position/pose", 10, local_pose_cb);
	ros::Subscriber local_target_pub = nh.subscribe<geometry_msgs::Pose2D>("/drone1/global_to_local_converted",10, local_targetpose_cb);
	// master and ROI topics
	// ros::Subscriber groundStation_value = nh.subscribe<swarm_search::sip_goal>("master/drone1/ground_msg",10,callback_sip);
	ros::Subscriber groundStation_value2 = nh.subscribe<swarm_search::sip_goal2>("master/drone1/ground_msg",10,callback_sip2);
	ros::Subscriber ROI_points = nh.subscribe<swarm_search::point_list>("/drone1/ROI_flow",10,roi_list);
	//Diagonostic Feedback Topic 

	// allow the subscribers to initialize
	flags.scan_flag.data = 0;
	flags.transition_s2s.data = 0;
	flags.search_flag.data = 0;
	flags.recovery1_flag.data = 0;
	flags.recovery2_flag.data = 0;

	ROS_INFO("INITIALISING...");
	for(int i=0; i<50; i++)
	{
		ros::spinOnce();
		ros::Duration(0.01).sleep();
	}

	// change mode to GUIDED
	while(current_state.mode != "GUIDED")
	{
		ros::spinOnce();
		ros::Duration(0.01).sleep();
	}

	// // wait for FCU connection
	while(ros::ok() && current_state.mode == "GUIDED")
	{
		// // wait for FCU connection
		while (ros::ok() && !current_state.connected)
		{
			ros::spinOnce();
			rate.sleep();
		}

		while(!takeoff_done && ros::ok())
		{
			while(!armed  && ros::ok() && current_state.mode == "GUIDED")
			{
				armed = arm_drone(nh);
				cout << "MODE CHANGE TO GUIDED : stuck here" << endl; //debug statement
				ros::spinOnce();
			}
		 
			while( master_goal2.takeoff_flag.data != true  && ros::ok() && current_state.mode == "GUIDED")
			{
				cout <<"**********GUIDED*************" <<master_goal2.takeoff_flag.data << endl;
				armed = arm_drone(nh);
				cout << master_goal2.takeoff_flag.data << endl;
				ROS_INFO("Waiting for the permission to take off");
				ros::spinOnce();
				ros::Duration(0.01).sleep();
				//wait for permission to take off
			}
		
			ros::ServiceClient takeoff_cl = nh.serviceClient<mavros_msgs::CommandTOL>("/drone1/mavros/cmd/takeoff");
			mavros_msgs::CommandTOL srv_takeoff;
			srv_takeoff.request.altitude = takeoff_alt;
			
			if(takeoff_cl.call(srv_takeoff)){
				ROS_INFO("takeoff sent %d", srv_takeoff.response.success);
				takeoff_done = 1;
			 }
			 
			 else
			 {
				ROS_ERROR("Failed Takeoff");
				return -1;
			 }

			sleep(5);
			}
			ros::spinOnce();
			scan_main(nh);
		}
}
