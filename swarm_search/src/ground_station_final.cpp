#include <cmath>
#include "ros/ros.h"
#include <math.h>
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Point.h"
#include <geometry_msgs/Twist.h>
#include "waypoint_generator/point_list.h"
#include "geometry_msgs/PoseStamped.h"
#include <std_msgs/Float64.h>
#include <swarm_search/local_flags.h>

using namespace std;
using namespace ros;

ros::Publisher waypoint_pub;
swarm_search::point_list detected_points;

bool target0 = false;
void targets_cb0(const swarm_search::point_list indi_points)
{
	if(!target0 && flags.recovery1_flag.data == true)
	{
		target0 = true;
		for (int i = 0; i < indi_points.points.size(); ++i)
		{
			detected_points.points.push_back(indi_points.points[i]);
		}

	}

}

bool target1 = false;
void targets_cb1(const swarm_search::point_list indi_points)
{
	if(!target1 && flags.recovery1_flag.data == true)
	{
		target1 = true;
		cout<<"Points of drone1 added:"<<indi_points.points.size()<<endl;
		for (int i = 0; i < indi_points.points.size(); ++i)
		{
			detected_points.points.push_back(indi_points.points[i]);
		}

	}

}

bool target2 = false;
void targets_cb2(const swarm_search::point_list indi_points)
{
	if(!target2 && flags.recovery1_flag.data == true)
	{
		target2 = true;
		cout<<"Points of drone2 added:"<<indi_points.points.size()<<endl;
		for (int i = 0; i < indi_points.points.size(); ++i)
		{
			detected_points.points.push_back(indi_points.points[i]);
		}

	}

}

bool target3 = false;
void targets_cb3(const swarm_search::point_list indi_points)
{
	if(!target3 && flags.recovery1_flag.data == true)
	{
		target3 = true;
		cout<<"Points of drone3 added:"<<indi_points.points.size()<<endl;
		for (int i = 0; i < indi_points.points.size(); ++i)
		{
			detected_points.points.push_back(indi_points.points[i]);
		}

	}

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

waypoint_generator::point_list corrected_frame_points;
void corrected_targets_cb(const waypoint_generator::point_list::ConstPtr& frame_points)
{
	if( flags.recovery1_flag.data == false && target0 && target1 && target2 && target3)
		return;

	corrected_frame_points.points.clear();
	for (int i = 0; i < frame_points->size(); ++i)
	{
		corrected_frame_points.points.push_back(frame_points->points[i]);
	}

	for (int j = 0; j < corrected_frame_points.points.size(); ++j)
	{
		for (int i = 0; i < detected_points.points.size(); ++i)
		{
			if( haversine(detected_points.points[i].x, detected_points.points[i].y, 
				corrected_frame_points.points[j].x, corrected_frame_points.points[j].y) < 0.010 )
			{
				corrected_frame_points.points[j].x = detected_points.points[i].x;
				corrected_frame_points.points[j].y = detected_points.points[i].y;
			}
		}
	}
}



int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "ground_final");
	ros::NodeHandle n;
	
  	ros::Subscriber status_sub = n.subscribe("/drone1/flags", 1000, statusCallback);
	ros::Subscriber closed_frame_point_sub = n.subscribe("/master/corrected_coordinates", 1000, corrected_targets_cb);
	
	ros::Subscriber target_sub0 = nh.subscribe<swarm_search::point_list>("/drone0/listener1",10,targets_cb0);
	ros::Subscriber target_sub1 = nh.subscribe<swarm_search::point_list>("/drone1/listener1",10,targets_cb1);
	ros::Subscriber target_sub2 = nh.subscribe<swarm_search::point_list>("/drone2/listener1",10,targets_cb2);
	ros::Subscriber target_sub3 = nh.subscribe<swarm_search::point_list>("/drone3/listener1",10,targets_cb3);

  	waypoint_pub = n.advertise<geometry_msgs::Point>("/drone1/listener", 1000);

    ros::Rate loop_rate(20);
    while( ros::ok )
    {	
		if( flags.recovery2_flag.data == true)
			waypoint_pub.publish(corrected_frame_points);
		
		ros::spinOnce();
		loop_rate.sleep();
    }

	return 0;
}

