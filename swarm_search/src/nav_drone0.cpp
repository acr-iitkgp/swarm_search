#include <iostream>
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
#include "std_msgs/Float64.h"
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <unistd.h>
#include <time.h>
#include <cmath>
#include <math.h>

using namespace std;

//Set global variables
sensor_msgs::NavSatFix current_pose;
mavros_msgs::GlobalPositionTarget pose;

double goal_tolerance = 0.002;

double delta_to_destination;
int time_tolerance_to_destination = 100000;

geometry_msgs::PoseStamped current_pose; //current drone pose
geometry_msgs::PoseStamped pose; //target pose - updated in function setDestination

geometry_msgs::Twist drone_status;
geometry_msgs::Twist drone_diag;
geometry_msgs::Twist received_cmd;

void setDestination(const mavros_msgs::GlobalPositionTarget::ConstPtr& msg2)
{
  pose = *msg2;
  double X,Y,Z;

  X = pose.latitude;
  Y = pose.longitude;
  Z = pose.altitude;
  ROS_INFO("Destination set to Latitude: %f Longitude: %f Altitude %f", X, Y, Z);
}

double haversine(double lat1, double lon1, double lat2, double lon2) //distance between two global
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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "drone0_nav_node");
  ros::NodeHandle nh;
  int k = navigate(nh);

  // // the setpoint publishing rate MUST be faster than 2Hz
  // ros::Rate rate(20.0);
  // // ros::Publisher set_vel_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
  // ros::Publisher global_pos_pub = nh.advertise<mavros_msgs::GlobalPositionTarget>("mavros/setpoint_position/global", 10);
  // // ros::Subscriber currentPos = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/global_position/pose", 10, pose_cb);
  // ros::Subscriber currentPos = nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 10, pose_cb);
  // // ros::Subscriber currentHeading = nh.subscribe<std_msgs::Float64>("/mavros/global_position/compass_hdg", 10, heading_cb);
  // ros::Subscriber nextGoal = nh.subscribe<mavros_msgs::GlobalPositionTarget>("/master/drone0/next_goal/pose", 10, setDestination);

  // // allow the subscribers to initialize
  // ROS_INFO("INITIALISING...");
  // for(int i=0; i<100; i++)
  // {
  //   ros::spinOnce();
  //   ros::Duration(0.01).sleep();
  // }

  // if(global_pos_pub)
  // {    //to reach the destination, it returns 1 when it has reached the destination
  //   for (int i=time_tolerance_to_destination; ros::ok() && i>0;--i)
  //   {
  //     global_pos_pub.publish(pose);
  //     delta_to_destination = haversine(pose.latitude,pose.longitude,current_pose.latitude,current_pose.longitude);
  //     cout << "Delta to Destination : "<< delta_to_destination << endl;
  //     if( delta_to_destination < goal_tolerance )
  //     {
  //       ROS_INFO("Reached at the target position");  
  //       return 1;
  //     }
  //   }

  // while (ros::ok())
  // {
  //   ros::spinOnce();
  //   rate.sleep();
  // }
  // return 0;

// }
}

int navigate(ros::NodeHandle nh)
{
  // the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(20.0);
  // ros::Publisher set_vel_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
  ros::Publisher global_pos_pub = nh.advertise<mavros_msgs::GlobalPositionTarget>("mavros/setpoint_position/global", 10);
  // ros::Subscriber currentPos = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/global_position/pose", 10, pose_cb);
  ros::Subscriber currentPos = nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 10, pose_cb);
  // ros::Subscriber currentHeading = nh.subscribe<std_msgs::Float64>("/mavros/global_position/compass_hdg", 10, heading_cb);
  ros::Subscriber nextGoal = nh.subscribe<mavros_msgs::GlobalPositionTarget>("/master/drone0/next_goal/pose", 10, setDestination);

  // allow the subscribers to initialize
  ROS_INFO("INITIALISING...");
  for(int i=0; i<100; i++)
  {
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }

  if(global_pos_pub)
  {    //to reach the destination, it returns 1 when it has reached the destination
    for (int i=time_tolerance_to_destination; ros::ok() && i>0;--i)
    {
      global_pos_pub.publish(pose);
      delta_to_destination = haversine(pose.latitude,pose.longitude,current_pose.latitude,current_pose.longitude);
      cout << "Delta to Destination : "<< delta_to_destination << endl;
      if( delta_to_destination < goal_tolerance )
      {
        ROS_INFO("Reached at the target position");  
        return 1;
      }
    }

  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
}