#include <iostream>
#include "geodetic_conv.hpp"


int main()
{
	double lat = 50, lon = 50, alt = 0;
	geodetic_converter::GeodeticConverter conv;
	conv.initialiseReference(lat,lon,alt);
	// double l = 2*height*tan(FOV/2); //metres
	// double length_per_pixel = l/resolution_y;
	double lat_temp, lon_temp, alt_temp;
	double distance_N, distance_E;
	// geometry_msgs::Point temp_point;
	// for (int i=0; i<frame_points->points.size(); i++)
	// {
		distance_N = 100;//(frame_points->points[i].x-resolution_x/2)*length_per_pixel;
		distance_E = 0;//(frame_points->points[i].y-resolution_y/2)*length_per_pixel;
		conv.enu2Geodetic(distance_E, distance_N, 0, &lat_temp, &lon_temp, &alt_temp);
		// temp_point.x = lat_temp;
		// temp_point.y = lon_temp;
		cout<<lat_temp<<" "<<lon_temp<<endl;
		// frame_points_gps.points.push_back(temp_point);
	// }	
}
