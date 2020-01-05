#include <cmath>
#include "ros/ros.h"
#include <math.h>
#include "geodetic_conv.hpp"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Point.h"
#include <geometry_msgs/Twist.h>
#include "waypoint_generator/point_list.h"
#include "geometry_msgs/PoseStamped.h"
#include <std_msgs/Float64.h>

// #define M_PI 3.14159

using namespace std;
using namespace ros;

// nav_msgs::Path path;
bool waypoint_reached = true;
double height = 0;
double FOV = 2*M_PI/3;  			// define in radians 
double resolution_y = 1280;		// image cols
double resolution_x = 720;		// image rows

bool GPS_received, heading_received, height_received, frameToGPS;

geometry_msgs::PoseStamped current_pos;

std::map < pair<double, double>, vector<pair<double, double> > > ROI_list;

// GPS coordinate received 
double lat, lon, alt;
void globalCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
	// if(!GPS_received)
 	lat=msg->latitude;
 	lon=msg->longitude;
 	alt=msg->altitude;
	// cout<<"GPS received: "<<lat<<" "<<lon<<" "<<alt<<endl; 	
}

// heading angle received
double heading_angle = 0.0;
void orientCallback(const std_msgs::Float64::ConstPtr& msg)
{
	heading_angle = msg->data * M_PI/180.0;
	// cout<<"Heading received: "<<heading_angle<<endl;
}

// height callback
void localposcallback(const geometry_msgs::PoseStamped::ConstPtr& position)
{
	current_pos = *position;
	height = current_pos.pose.position.z;
	// cout<<"Height received: "<<height<<endl;

}

// input is list of frame points 
// converting that to distance in m
// finding cosine in direction of N and E 
// using N and E to find 

waypoint_generator::point_list frame_points_gps;
void framePointCallback(const waypoint_generator::point_list::ConstPtr& frame_points)
{
	geodetic_converter::GeodeticConverter conv;
	conv.initialiseReference(lat,lon,alt);
	
	// cout<<"length_per_pixel: "<<length_per_pixel<<endl;
	double alpha = 0;
	double lat_temp, lon_temp, alt_temp;
	double distance_N, distance_E, distance;
	double length_per_pixel = 2*height*tan(FOV/2)/resolution_y;
	
	geometry_msgs::Point temp_point;
	for (int i=0; i<frame_points->points.size(); i++)
	{
		distance = length_per_pixel*sqrt(pow(frame_points->points[i].x-resolution_x/2, 2)+(frame_points->points[i].y-resolution_y/2, 2));
		// cout<<"distance: "<<distance<<endl;
		if((frame_points->points[i].x-resolution_x/2)!= 0)
			alpha = atan((frame_points->points[i].y-resolution_y/2)/(frame_points->points[i].x-resolution_x/2));
		else if((frame_points->points[i].y-resolution_y/2)>0)
			alpha = M_PI/2;
		else if((frame_points->points[i].y-resolution_y/2)<0 )
			alpha = -M_PI/2;
		else alpha = 0; 
		// cout<<"α: "<<alpha<<endl;
		if (frame_points->points[i].x-resolution_x/2 < 0)
			distance *= (-1);
		distance_E = distance*sin(M_PI-alpha+heading_angle);
		distance_N = distance*cos(M_PI-alpha+heading_angle);    
		// cout<<distance_N<<" "<<distance_E<<endl; 

		conv.enu2Geodetic(distance_E, distance_N, 0, &lat_temp, &lon_temp, &alt_temp);
		temp_point.x = lat_temp;
		temp_point.y = lon_temp;
		// cout << std::fixed << std::setprecision(8)<< "Lat: "<<lat_temp<<" Lon: "<<lon_temp<<endl;
		frame_points_gps.points.push_back(temp_point);
	}	

	double scale = 0.00001;
	for (int i=0; i<frame_points_gps.points.size(); i++)
	{
		double lat = frame_points_gps.points[i].x;
		double lon = frame_points_gps.points[i].y; 
		double lat_scaled = (int)(lat/scale) * scale;
		double lon_scaled = (int)(lon/scale) * scale;
		ROI_list[{lat_scaled, lon_scaled}].push_back({lat, lon});
	}
}

ros::Publisher waypoint_pub;
void statusCallback(const geometry_msgs::Twist::ConstPtr& data)
{
	if (data->linear.y == 1)
	{
		waypoint_generator::point_list final_target_points_gps;
		for (auto i=ROI_list.begin(); i!=ROI_list.end(); i++)
		{
			geometry_msgs::Point temp;
			double lat_final=0, lon_final=0;
			for (int j=0; j<i->second.size(); j++)
			{
				lat_final += i->second[j].first;
				lon_final += i->second[j].second;
			}
			lat_final /= i->second.size();
			lon_final /= i->second.size();
			temp.y = lon_final;
			temp.x = lat_final;
			final_target_points_gps.points.push_back(temp);
			// cout<<"Final points: "<< std::fixed << std::setprecision(8)<<temp.x<<" "<<temp.y<<endl;
		}
		waypoint_pub.publish(final_target_points_gps);
	}
}

int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "ROI_publisher");
	ros::NodeHandle n;
	
	ros::Subscriber frame_point_sub = n.subscribe("/drone1/some_topic", 1000, framePointCallback);
	ros::Subscriber gps_sub = n.subscribe("/drone1/mavros/global_position/global", 10, globalCallback);
	ros::Subscriber current_position = n.subscribe("/drone1/mavros/local_position/pose",10, localposcallback);
  	ros::Subscriber heading_sub = n.subscribe("/drone1/mavros/global_position/compass_hdg", 1000, orientCallback);
  	ros::Subscriber status_sub = n.subscribe("/drone1/flags", 1000, statusCallback);
  	waypoint_pub = n.advertise<waypoint_generator::point_list>("/drone1/ROI_flow", 1000);

    ros::Rate loop_rate(20);
    while( ros::ok )
    {			
		ros::spinOnce();
		loop_rate.sleep();
    }

	return 0;
}





// input: M_PIxel_points, drone_gps, theta, height, image_resolution(x,y) 