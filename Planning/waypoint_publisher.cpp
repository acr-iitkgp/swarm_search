#include <cmath>
#include <fstream>
#include "ros/ros.h"

using namespace std;
using namespace ros;

nav_msgs::Path path;
int curr_waypoint = 0;
bool waypoint_reached = true;

void locationCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{

	float distance = pow(msg.pose.position.x-path[curr_waypoint].pose.position.x,2)
					+pow(msg.pose.position.y-path[curr_waypoint].pose.position.y,2)
					+pow(msg.pose.position.z-path[curr_waypoint].pose.position.z,2);			

	if( sqrt(distance)< distanceThresh )
		waypoint_reached = true;
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "waypoint_publisher");
	ros::NodeHandle n;
	ros::Publisher way_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000);
	ros::Subscriber way_sub = n.subscribe("/some_topic", 1000, locationCallback);
	ros::Rate loop_rate(10);

	string frame = "frame";
	path.header.frame_id = frame; 			//change
	path.header.stamp = ros::Time::now();

	// convert list of waypoints to PoseStamped array
	ifstream inFile;
	inFile.open("My_Path.txt");
	
	float x = 0, y = 0, z = 0;
	while (inFile >> x >>y >>z )
    {
		geometry_msgs::PoseStamped waypoint;
		waypoint.header.frame_id = frame;
		waypoint.pose.position.x = x;
		waypoint.pose.position.y = y;
		waypoint.pose.position.z = z;
		
		waypoint.pose.orientation.x = 0.0;
		waypoint.pose.orientation.y = 0.0;
		waypoint.pose.orientation.z = 0.0;
		waypoint.pose.orientation.w = 0.0;

		path.poses.push_back(waypoint);
    
    }

	// depending on the reached signal publish next waypoint
    ros::Rate loop_rate(10);
    while( ros::ok )
    {	
    	if( waypoint_reached )
		{
			waypoint_reached = false;
			way_pub.publish(path[curr_waypoint]);
			curr_waypoint++;
		}	
		
		ros::spinOnce();
		loop_rate.sleep();
    }


	return 0;
}