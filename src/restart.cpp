#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/Float64.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/CommandCode.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <unistd.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/CommandCode.h>
#include <mavros_msgs/PositionTarget.h>
#include <sensor_msgs/Range.h>
#include <time.h>
#include <cmath>
#include <math.h>
#include <ros/duration.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <queue>
using namespace std;
using namespace mavros_msgs;
#define Point_P_x 0.0012
#define Point_D_x 0 //0.001
#define Point_P_y 0.0012
#define Point_D_y 0 //0.001
//Set global variables
mavros_msgs::State current_state;

/***************************************call backs********************************************/
//get state
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
	current_state = *msg;
	bool connected = current_state.connected;
	bool armed = current_state.armed;
}
/***************************************call backs end****************************************/

int restart_drone(ros::NodeHandle &nh)
{
	// arming
	ros::ServiceClient restart_client_i = nh.serviceClient<mavros_msgs::CommandLong>("mavros/cmd/command");
	mavros_msgs::CommandLong srv_restart_i;
	srv_restart_i.request.command = mavros_msgs::CommandCode::PREFLIGHT_REBOOT_SHUTDOWN;
	srv_restart_i.request.param1 = 1;
	restart_client_i.call(srv_restart_i);
	// if (restart_client_i.call(srv_restart_i) && srv_restart_i.response.success)
	// 	ROS_INFO("Restart sent %d", srv_restart_i.response.success);
	// else
	// {
	// 	ROS_ERROR("Failed restart");
	// 	return -1;
	// }
	return 0;
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "restart_FCU");
	ros::NodeHandle nh;
	// the setpoint publishing rate MUST be faster than 2Hz
	ros::Rate rate(20.0);
	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);
	// allow the subscribers to initialize
	for (int i = 0; i < 100; i++)
	{
		ros::spinOnce();
		ros::Duration(0.01).sleep();
	}
	// recheck for FCU connection
	while (ros::ok() && !current_state.connected)
	{
		ros::spinOnce();
		rate.sleep();
	}

	restart_drone(nh);

	return 0;
}
