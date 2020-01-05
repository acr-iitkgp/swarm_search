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

#include <swarm_search/sip_goal.h>
#include <swarm_search/point_list.h>
#include <swarm_search/local_flags.h>

using namespace std;

//Set global variables
mavros_msgs::State current_state;

double takeoff_alt = 1.5;
double search_altitude = 2.0;
double scan_altitude = 5.0;
double goal_tolerance = 0.002;

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

sensor_msgs::NavSatFix current_pose; //current drone pose
geographic_msgs::GeoPoseStamped pose; //target pose - updated in function setDestination
geometry_msgs::Quaternion current_orientation, average_orientation;

swarm_search::point_list arr;
swarm_search::sip_goal master_goal;
swarm_search::local_flags flags;

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
  if(counter_1<15)
  {
    initial_takeoff_height += current_pose.altitude;
    counter_1 += 1;
  }
  else if( counter_1 == 15)
  {
	initial_takeoff_height = initial_takeoff_height/15;
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


void callback_sip(const swarm_search::sip_goal::ConstPtr& msg)
{
  master_goal = *msg;
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
    double rad = 6371; 
    double c = 2 * asin(sqrt(a)); 
    return rad * c; 
} 

int navigate(ros::NodeHandle nh, geographic_msgs::GeoPoseStamped pose1)
{
  // the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(10.0);

  cout << pose1 << endl;

  ros::Publisher global_pos_pub = nh.advertise<geographic_msgs::GeoPoseStamped>("/drone3/mavros/setpoint_position/global_to_local", 10);
  ros::Subscriber currentPos = nh.subscribe<sensor_msgs::NavSatFix>("/drone3/mavros/global_position/global", 10, pose_cb);

  //DRONE DIAG FLAGS TO BE ADDED 

  // allow the subscribers to initialize
  ROS_INFO("INITIALISING...");
  for(int i=0; i<100; i++)
  {
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }

  pose1.header.stamp = ros::Time::now();
  global_pos_pub.publish(pose1);

  int tolerance_time = 0;

  if(global_pos_pub)
  {  //to reach the destination, it returns 1 when it has reached the destination
    // for (int i=time_tolerance_to_destination; ros::ok() && i>0;--i)
    while(ros::ok())
    {
      delta_to_destination = haversine(pose1.pose.position.latitude,pose1.pose.position.longitude,current_pose.latitude,current_pose.longitude);

      ROS_INFO("Delta to Destination: %f  Delta to Altitude: %f ", delta_to_destination, abs(pose1.pose.position.altitude-current_pose.altitude));

      pose1.header.stamp = ros::Time::now();
      global_pos_pub.publish(pose1);

      if(abs(delta_to_destination - prev_delta) < 0.000005 && abs(pose1.pose.position.altitude-current_pose.altitude)<0.7)
      {
        tolerance_time++;
        ROS_INFO("tolerance time is : %d", tolerance_time);
      }


      if( ((delta_to_destination < goal_tolerance) && abs(pose1.pose.position.altitude-current_pose.altitude)<0.5) ||tolerance_time > 150) /////////////////////change this//////////
      {
        ROS_INFO("Reached at the target position");  
        ROS_INFO("tolerance time is : %d", tolerance_time);
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
  ros::ServiceClient arming_client_i = nh.serviceClient<mavros_msgs::CommandBool>("/drone3/mavros/cmd/arming");
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
    ros::ServiceClient takeoff_cl = nh.serviceClient<mavros_msgs::CommandTOL>("/drone3/mavros/cmd/takeoff");
    mavros_msgs::CommandTOL srv_takeoff;
    srv_takeoff.request.altitude = takeoff_alt_local;
    if(takeoff_cl.call(srv_takeoff)){
      ROS_INFO("takeoff sent %d", srv_takeoff.response.success);
      sleep(10);
      return 1;
    }
    else{
      ROS_ERROR("Failed Takeoff");
      return 0;
    }
    sleep(10);

}

bool land(ros::NodeHandle nh)
{
  ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("/drone3/mavros/cmd/land");
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


int search_main(ros::NodeHandle nh)
{
  ros::Publisher flags_pub = nh.advertise<swarm_search::local_flags>("/drone3/flags", 10);
  int k = 0;

  for(int i=0;arr.points[i].x!=0;i++)
  {
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "target_position";
    pose.pose.position.latitude = arr.points[i].x;
    pose.pose.position.longitude = arr.points[i].y;
    pose.pose.position.altitude = initial_takeoff_height + search_altitude;
    pose.pose.orientation = current_orientation;
    k = navigate(nh,pose);

    flags.search_flag.data = 1;
    for (int j= 0; j < 5; ++j)
    {
      flags_pub.publish(flags);
      sleep(0.1);
    }

    sleep(5);

    flags.search_flag.data = 0; 
    for (int j= 0; j < 5; ++j)
    {
      flags_pub.publish(flags);
      sleep(0.1);
    }
  
    // int detected_points = 0;
    // if(detected_points >=4)
    // {
    //   return 1;
    // }
  }

  //exit ke baad flag change hona chahiye code me

  ROS_INFO("Main search is now over");
  return 0;

  // ek callback hoga yaha se that will take ROI points and go there.

  // multi goal path planning se waypoints leke list me rakaho
  // ek ek kar ke navigate(pose) : pose update karte raho when reached prev target
  // sleep for some time so that sufficient frames can be taken
  // keep checking # of target detection, if = 4 : end search  
}


void scan_main(ros::NodeHandle nh)
{

  ros::Publisher flags_pub = nh.advertise<swarm_search::local_flags>("/drone3/flags", 10);
  
  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = "target_position";
  pose.pose.position.latitude = master_goal.sip_start.x;
  pose.pose.position.longitude = master_goal.sip_start.y;
  pose.pose.position.altitude = initial_takeoff_height + scan_altitude;
  pose.pose.orientation = current_orientation;

  int x = navigate(nh, pose); // returns 1 when reached
	
	if(x == 1)
	{
		ROS_INFO(" SIP Reached ");
		ROS_INFO(" Requesting ROI Scanning to start ");

    flags.scan_flag.data = 1; 
    for (int j= 0; j < 5; ++j)
    {
      flags_pub.publish(flags);
      sleep(0.1);
    } 
    //scanning should start here

   	sleep(5);

		pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "target_position";
		pose.pose.position.latitude = master_goal.sip_end.x;
		pose.pose.position.longitude = master_goal.sip_end.y;
		pose.pose.position.altitude = initial_takeoff_height + scan_altitude;
    pose.pose.orientation = current_orientation;

   	int y = navigate(nh, pose);

 		if(y == 1)
 		{
  		ROS_INFO(" Itteration 1 Scanning Complete ");
  		ROS_INFO(" Requesting ROI Scanning to stop and pass coordinated to Multi-Goal Path Planner ");

      flags.scan_flag.data = 0;//ROI_scan_flag;
      flags.transition_s2s.data = 1; // between scanning and search
      for (int j= 0; j < 5; ++j)
      {
        flags_pub.publish(flags);
        sleep(0.1);
      }

      int z = check_ROI_confidence(); // to be done, it analyses features

    	if (z == 1)
    	{	
        ROS_INFO(" Good ROI Confidence - Proceeding to Search and Verify Target ");

        sleep(2);
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "target_position";
        pose.pose.position.latitude = master_goal.sip_end.x;
        pose.pose.position.longitude = master_goal.sip_end.y;
        pose.pose.position.altitude = initial_takeoff_height + search_altitude;
        pose.pose.orientation = current_orientation;

        // start Multi-Goal Path Planning and wait for result, also start the detection code

        int w = navigate(nh,pose);
        sleep(1);

        //land(nh);
        //return;

        if (w==1)
        {
          ROS_INFO(" Reached the Search Height ");
          flags.transition_s2s.data = 0; //between scanning and search over
          for (int j= 0; j < 5; ++j)
          {
            flags_pub.publish(flags);
            sleep(0.1);
          }          
          int k = search_main(nh); // k = 1 means all detection is completed and all four are found
          // k = 0 means not all have been found but all roi has been explored
        }

    	  }
    	else
    	{
    		ROS_INFO(" Poor ROI Confidence - Requesting Recovery Mode 1 ");
    		flags.recovery1_flag.data = 1;//recovery1_flag = 1;
    		flags_pub.publish(flags);
    	}
 		}
 		else
 		{
 			ROS_INFO(" FAILED TO COMPLETE SCAN_1 ");
 		}
 	}

 	else
 	{
 		ROS_INFO(" FAILED TO REACH SIP ");
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
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/drone3/mavros/state", 10, state_cb);
  ros::Publisher global_pos_pub = nh.advertise<geographic_msgs::GeoPoseStamped>("/drone3/mavros/setpoint_position/global_to_local", 10);
  ros::Subscriber currentPos = nh.subscribe<sensor_msgs::NavSatFix>("/drone3/mavros/global_position/global", 10, pose_cb);
  ros::Subscriber localPos = nh.subscribe<geometry_msgs::PoseStamped>("/drone3/mavros/local_position/pose", 10, orientation_cb);


  // master and ROI topics
  ros::Subscriber groundStation_value = nh.subscribe<swarm_search::sip_goal>("master/drone3/ground_msg",10,callback_sip);
  ros::Subscriber ROI_points = nh.subscribe<swarm_search::point_list>("/drone3/ROI_flow",10,roi_list);
 
  //Diagonostic Feedback Topic 
  ros::Publisher drone_status_pub = nh.advertise<geometry_msgs::Twist>("/master/drone3/drone_status", 10);

 
  // allow the subscribers to initialize

  flags.scan_flag.data = 0;
  flags.transition_s2s.data = 0;
  flags.search_flag.data = 0;
  flags.recovery1_flag.data = 0;
  flags.recovery2_flag.data = 0;

  ROS_INFO("INITIALISING...");
  for(int i=0; i<100; i++)
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
   
    while( master_goal.takeoff_flag.data != true  && ros::ok() && current_state.mode == "GUIDED")
    {
      cout <<"**********GUIDED*************" <<master_goal.takeoff_flag.data << endl;
      armed = arm_drone(nh);
      cout << master_goal.takeoff_flag.data << endl;
      ROS_INFO("Waiting for the permission to take off");
      ros::spinOnce();
      ros::Duration(0.01).sleep();
      //wait for permission to take off
    }
  
  ros::ServiceClient takeoff_cl = nh.serviceClient<mavros_msgs::CommandTOL>("/drone3/mavros/cmd/takeoff");
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

