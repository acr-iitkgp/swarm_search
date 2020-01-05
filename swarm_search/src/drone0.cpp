#include <ros/ros.h>
#include <ros/duration.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandTOL.h>

#include <nav_msgs/Odometry.h>
#include "std_msgs/Float64.h"
#include <unistd.h>
#include <time.h>
#include <cmath>
#include <math.h>
#include <iostream>

using namespace std;


//Set global variables
mavros_msgs::State current_state;

geometry_msgs::PoseStamped current_pose;
geometry_msgs::PoseStamped pose;
// std_msgs::Float64 current_heading;

bool connected;
bool armed;
float goal_tolerance = .35;

//get armed state
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
    connected = current_state.connected;
    armed = current_state.armed;
}

//get current position of drone
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  current_pose = *msg;
  ROS_INFO("x: %f y: %f z: %f", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
}


//void setDestination(const geometry_msgs::PoseStamped::ConstPtr& msg2)
void setDestination(float X, float Y, float Z)
{
    //destination_pose = *msg2;

    //ROS_INFO("DESTINATION : x: %f y: %f z: %f", destination_pose.pose.position.x, destination_pose.pose.position.y, destination_pose.pose.position.z);

    //conversion to END Coordinate System
    //float deg2rad = (M_PI/180);
    
    //perform any conversion required
    //float X = destination_pose.pose.position.x;//x*cos(-GYM_OFFSET*deg2rad) - y*sin(-GYM_OFFSET*deg2rad);
    //float Y = destination_pose.pose.position.y;//x*sin(-GYM_OFFSET*deg2rad) + y*cos(-GYM_OFFSET*deg2rad);
    //float Z = destination_pose.pose.position.z;//z;
    
    //set destination coordinates
    pose.pose.position.x = X;
    pose.pose.position.y = Y;
    pose.pose.position.z = Z;
    ROS_INFO("Destination set to x: %f y: %f z %f", X, Y, Z);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "drone0_offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
 
 	ros::Subscriber currentPos = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/global_position/pose", 10, pose_cb);
	//ros::Subscriber currentHeading = nh.subscribe<std_msgs::Float64>("/mavros/global_position/compass_hdg", 10, heading_cb);

    //ros::Subscriber nextGoal = nh.subscribe<geometry_msgs::PoseStamped>("/master/drone0/next_goal/pose", 10, setDestination);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("Drone 0 NOT CONNECTED!");
    }

	// allow the subscribers to initialize
	ROS_INFO("INITIALISING...");
	for(int i=0; i<100; i++)
	{
	   ros::spinOnce();
	   ros::Duration(0.01).sleep();
	}
	
	// while(current_state.mode != "GUIDED")
	// {
	// 	ros::spinOnce();
	// 	ros::Duration(0.01).sleep();
	// }

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok())
    {
       	//checking OFFBOARD and ARMED Status 
        if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                ROS_INFO("OFFBOARD ENABLED!");
            }
            last_request = ros::Time::now();

        } 

        else 
        {
            if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) && arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
    			//else{
				// 	ROS_ERROR("Failed arming");
				// 	return -1;
				// }
                last_request = ros::Time::now();
            }
        }

    //MISSION CMD

        //local_pos_pub.publish(pose);

        //request takeoff
        ros::ServiceClient takeoff_cl = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
        mavros_msgs::CommandTOL srv_takeoff;
        srv_takeoff.request.altitude = 1.5;
        if(takeoff_cl.call(srv_takeoff))
        {
            ROS_INFO("takeoff sent %d", srv_takeoff.response.success);
        }
        else
        {
            ROS_ERROR("Failed Takeoff");
            return -1;
        }

        sleep(10);


        //move foreward
        //setHeading(0);
        setDestination(0, 2, 1.5);
        
        if (local_pos_pub)
        {
            for (int i = 10000; ros::ok() && i > 0; --i)
            {

                local_pos_pub.publish(pose);

                float deltaX = abs(pose.pose.position.x - current_pose.pose.position.x);
                float deltaY = abs(pose.pose.position.y - current_pose.pose.position.y);
                float deltaZ = abs(pose.pose.position.z - current_pose.pose.position.z);
                //cout << " dx " << deltaX << " dy " << deltaY << " dz " << deltaZ << endl;
                float dMag = sqrt( pow(deltaX, 2) + pow(deltaY, 2) + pow(deltaZ, 2) );
                cout << "Delta to Destination : "<< dMag << endl;
                if( dMag < goal_tolerance)
                {
                    break;
                }
                ros::spinOnce();
                ros::Duration(0.5).sleep();
                
                if(i == 1)
                {
                    ROS_INFO("Failed to reach destination. Stepping to next task.");
                }
            }
            ROS_INFO("Drone_0 Reached Destination.");
        }


        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}