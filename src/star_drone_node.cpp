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
#include <std_msgs/Byte.h>
#include <queue>
using namespace std;
using namespace mavros_msgs;
#define Point_P_x 0.0012
#define Point_D_x 0 //0.001
#define Point_P_y 0.0012
#define Point_D_y 0 //0.001
typedef struct
{
	double x;
	double y;
	double z;
	double yaw;
} local_point;
class PD_Controller
{
public:
	double P_x;
	double D_x;
	double P_y;
	double D_y;
	geometry_msgs::Point compute(geometry_msgs::Point error);
	PD_Controller(double param_P_x, double param_D_x, double param_P_y, double param_D_y);
	void reset();

private:
	double last_error_x;
	double last_error_y;
};
PD_Controller::PD_Controller(double param_P_x, double param_D_x, double param_P_y, double param_D_y) : P_x(param_P_x), D_x(param_D_x), P_y(param_P_y), D_y(param_D_y), last_error_x(0), last_error_y(0)
{
}
void PD_Controller::reset()
{
	last_error_x = 0;
	last_error_y = 0;
}
geometry_msgs::Point PD_Controller::compute(geometry_msgs::Point error)
{
	geometry_msgs::Point output;
	output.z = 0;
	output.x = error.x * P_x;
	output.y = error.y * P_y;
	double diff_x = error.x - last_error_x;
	double diff_y = error.y - last_error_y;
	output.x += diff_x * D_x;
	output.y += diff_y * D_y;
	last_error_x = error.x;
	last_error_y = error.y;
	return output;
}

//Set global variables
mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;
geometry_msgs::PoseStamped pose;
std_msgs::Float64 current_heading;
geometry_msgs::PoseStamped pose_snapshot;
geometry_msgs::Point current_point;
geometry_msgs::Point current_cam_front;
sensor_msgs::Range front_dis;
geometry_msgs::Point current_line;
double confidence_circle = 0;
double confidence_line = 0;
bool point_avalible = false;
bool line_avalible = false;
std_msgs::UInt8 line_seq;
float GYM_OFFSET;
tf2_ros::Buffer tfBuffer;
ros::Publisher set_raw_pub;
ros::Publisher local_pos_pub;
ros::Publisher mode_pub;
ros::Publisher capture_pub;
int red_count = 0;

std_msgs::Byte mptocv;
pthread_mutex_t mutex_cam_data;
pthread_mutex_t mutex_t265;
pthread_mutex_t mutex_cvtomp;
geometry_msgs::PoseStamped ctrl_cv;
char cvstate=0;
float posx_t265_local;
float posy_t265_local;
float posz_t265_local;
/***************************************call backs********************************************/
//get state
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
	current_state = *msg;
	bool connected = current_state.connected;
	bool armed = current_state.armed;
}

//get current position of drone
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	current_pose = *msg;
	//ROS_INFO("x: %f y: %f z: %f", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
	tf2::Quaternion q(
		current_pose.pose.orientation.x,
		current_pose.pose.orientation.y,
		current_pose.pose.orientation.z,
		current_pose.pose.orientation.w);
	tf2::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	current_heading.data = yaw;
	//	ROS_INFO("yaw: %f", current_heading.data);
}

void point_cb(const geometry_msgs::Point::ConstPtr& msg)
{
	current_point = *msg;
}

void line_cb(const geometry_msgs::Point::ConstPtr& msg)
{
	current_line = *msg;
}

void point_confi_cb(const std_msgs::Float64::ConstPtr& msg)
{
	confidence_circle = msg->data;
	point_avalible = true;
}

void line_confi_cb(const std_msgs::Float64::ConstPtr& msg)
{
	confidence_line = msg->data;
	line_avalible = true;
}

void camera_front_cb(const geometry_msgs::Point::ConstPtr& msg)
{
	current_cam_front = *msg;
}

void front_dis_cb(const sensor_msgs::Range::ConstPtr& msg)
{
	front_dis = *msg;
	//	ROS_INFO("range is %f",front_dis.range);
}

/******************call backs for zhx*****************/
void drone_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) //opencv
{
	pthread_mutex_lock(&mutex_cam_data);
	ctrl_cv = *msg;
	pthread_mutex_unlock(&mutex_cam_data);
	//	ROS_INFO("video:%.2f , %.2f", msg.pose.position.x, msg.pose.position.z);
}

void cvtomp_callback(const std_msgs::Byte::ConstPtr& msg)
{
	pthread_mutex_lock(&mutex_cvtomp);
	cvstate = msg->data;
	pthread_mutex_unlock(&mutex_cvtomp);
	//ROS_INFO("video:%.2f , %.2f", msg.pose.position.x, msg.pose.position.z);
}

void t265_callback(const nav_msgs::Odometry& msg)
{
	pthread_mutex_lock(&mutex_t265);
	posx_t265_local = - msg.pose.pose.position.x;
	posy_t265_local = - msg.pose.pose.position.y;
	posz_t265_local =   msg.pose.pose.position.z;
	pthread_mutex_unlock(&mutex_t265);
}
/******************call backs for zhx end*****************/

/***************************************call backs end****************************************/

//set orientation of the drone (drone should always be level)

/*
Heading sets the yaw of base_link in snapshot_takeoff in rad
base_link and snapshot_takeoff are both FLU frames.
Thus,the x axis of snapshot_takeoff is 0 rad.
must call send_tf_snapshot_takeoff before takeoff to take a snapshot.
*/
/*²»ÔÊÐí²»¶Ïµ÷ÓÃ£¬Ö»ÄÜµ÷ÓÃÒ»´ÎºóµÈÍê³ÉÔÙµ÷ÓÃ¡£*/
void set_pose_local(float x, float y, float z, float heading) //x y z is in reference to the body (base_link) frame captured when taking off
{
	heading = heading + GYM_OFFSET; //Sent pose gets transformed from ENU to NED later in order to be sent to FCU.
	float yaw = heading;			//YAW: from X(E) to Y(N) in ENU frame with X(E) axis being 0 deg
	float pitch = 0;				//level
	float roll = 0;					//level

	float cy = cos(yaw * 0.5);
	float sy = sin(yaw * 0.5);
	float cr = cos(roll * 0.5);
	float sr = sin(roll * 0.5);
	float cp = cos(pitch * 0.5);
	float sp = sin(pitch * 0.5);

	float qw = cy * cr * cp + sy * sr * sp;
	float qx = cy * sr * cp - sy * cr * sp;
	float qy = cy * cr * sp + sy * sr * cp;
	float qz = sy * cr * cp - cy * sr * sp;

	pose.pose.orientation.w = qw;
	pose.pose.orientation.x = qx;
	pose.pose.orientation.y = qy;
	pose.pose.orientation.z = qz;

	geometry_msgs::TransformStamped transformStamped;
	geometry_msgs::PointStamped initial_pt, transformed_pt;
	try
	{
		transformStamped = tfBuffer.lookupTransform("map", "snapshot_takeoff", ros::Time(0));
		initial_pt.point.x = x;
		initial_pt.point.y = y;
		initial_pt.point.z = z;
		tf2::doTransform(initial_pt, transformed_pt, transformStamped);
	}
	catch (tf2::TransformException& ex)
	{
		ROS_WARN("%s", ex.what());
		ros::Duration(1.0).sleep();
		return;
	}
	float X = transformed_pt.point.x;
	float Y = transformed_pt.point.y;
	float Z = transformed_pt.point.z;
	pose.pose.position.x = X;
	pose.pose.position.y = Y;
	pose.pose.position.z = Z;
	local_pos_pub.publish(pose);
	ROS_INFO("Destination set to x: %f y: %f z %f", X, Y, Z);
}

void send_tf_snapshot_takeoff(void)
{
	static tf2_ros::StaticTransformBroadcaster br;
	geometry_msgs::TransformStamped transformStamped;
	pose_snapshot = current_pose;
	;
	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = "map";
	transformStamped.child_frame_id = "snapshot_takeoff";
	transformStamped.transform.translation.x = pose_snapshot.pose.position.x;
	transformStamped.transform.translation.y = pose_snapshot.pose.position.y;
	transformStamped.transform.translation.z = pose_snapshot.pose.position.z;

	transformStamped.transform.rotation = pose_snapshot.pose.orientation;
	GYM_OFFSET = current_heading.data;
	br.sendTransform(transformStamped);
}

int arm_drone(ros::NodeHandle& nh)
{
	// arming
	ros::ServiceClient arming_client_i = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
	mavros_msgs::CommandBool srv_arm_i;
	srv_arm_i.request.value = true;
	if (arming_client_i.call(srv_arm_i) && srv_arm_i.response.success)
		ROS_INFO("ARM sent %d", srv_arm_i.response.success);
	else
	{
		ROS_ERROR("Failed arming");
		return -1;
	}
	return 0;
}

int set_servo(ros::NodeHandle& nh, bool state)//¶æ»ú
{
	ros::ServiceClient servo_client_i = nh.serviceClient<mavros_msgs::CommandLong>("mavros/cmd/command");
	mavros_msgs::CommandLong msg;
	msg.request.command = mavros_msgs::CommandCode::DO_SET_SERVO;
	msg.request.param1 = 9;
	msg.request.param2 = state ? 1900 : 1500;

	if (servo_client_i.call(msg) && msg.response.success)
		ROS_INFO("servo msg sent");
	else
	{
		ROS_ERROR("Failed servo");
		return -1;
	}
	return 0;
}

int takeoff(ros::NodeHandle& nh, double height) //meters
{
	ros::ServiceClient takeoff_cl = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
	mavros_msgs::CommandTOL srv_takeoff;
	srv_takeoff.request.altitude = height;
	if (takeoff_cl.call(srv_takeoff))
	{
		ROS_INFO("takeoff sent %d", srv_takeoff.response.success);
	}
	else
	{
		ROS_ERROR("Failed Takeoff");
		return -1;
	}
	return 0;
}

int land(ros::NodeHandle& nh)
{
	ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
	mavros_msgs::CommandTOL srv_land;
	if (land_client.call(srv_land) && srv_land.response.success)
		ROS_INFO("land sent %d", srv_land.response.success);
	else
	{
		ROS_ERROR("Landing failed");
		ros::shutdown();
		return -1;
	}
	return 0;
}

int get_Altitude_ControlMode(ros::NodeHandle& nh)//1 CMD
{
	// arming
	ros::ServiceClient mode_client_i = nh.serviceClient<mavros_msgs::CommandLong>("mavros/cmd/command");
	mavros_msgs::CommandLong msg;
	msg.request.command = 1;

	/*if (mode_client_i.call(msg) && msg.response.success)
	{
		ROS_INFO("mode is %d", msg.response.result_param2);
		return msg.response.result_param2;
	}
	else
	{
		ROS_ERROR("Failed mode");
		return -1;
	}*/
}

int get_Position_ControlMode(ros::NodeHandle& nh)
{
	// arming
	ros::ServiceClient mode_client_i = nh.serviceClient<mavros_msgs::CommandLong>("mavros/cmd/command");
	mavros_msgs::CommandLong msg;
	msg.request.command = 2;

	/*if (mode_client_i.call(msg) && msg.response.success)
	{
		ROS_INFO("mode is %d", msg.response.result_param2);
		return msg.response.result_param2;
	}
	else
	{
		ROS_ERROR("Failed mode");
		return -1;
	}*/
}

int set_speed_body(double x, double y, double z, double yaw_rate) //flu meter/s rad/s, must set continously, or the vehicle stops after a few seconds(failsafe feature). yaw_rate = 0 when not used.
{
	mavros_msgs::PositionTarget raw_target;
	raw_target.coordinate_frame = PositionTarget::FRAME_BODY_OFFSET_NED;
	raw_target.type_mask = PositionTarget::IGNORE_PX | PositionTarget::IGNORE_PY | PositionTarget::IGNORE_PZ | PositionTarget::IGNORE_AFX | PositionTarget::IGNORE_AFY | PositionTarget::IGNORE_AFZ | PositionTarget::IGNORE_YAW;
	if (fabs(yaw_rate) < 1e-6)
		raw_target.type_mask |= PositionTarget::IGNORE_YAW_RATE;
	raw_target.velocity.x = x;
	raw_target.velocity.y = y;
	raw_target.velocity.z = z;
	raw_target.yaw_rate = yaw_rate * 0.01745329;
	set_raw_pub.publish(raw_target);
	return 0;
}

int set_speed_enu(double x, double y, double z, double yaw_rate) //flu meter/s rad/s, must set continously, or the vehicle stops after a few seconds(failsafe feature). yaw_rate = 0 when not used.
{
	mavros_msgs::PositionTarget raw_target;
	raw_target.coordinate_frame = PositionTarget::FRAME_LOCAL_NED;
	raw_target.type_mask = PositionTarget::IGNORE_PX | PositionTarget::IGNORE_PY | PositionTarget::IGNORE_PZ | PositionTarget::IGNORE_AFX | PositionTarget::IGNORE_AFY | PositionTarget::IGNORE_AFZ | PositionTarget::IGNORE_YAW;
	if (fabs(yaw_rate) < 1e-6)
		raw_target.type_mask |= PositionTarget::IGNORE_YAW_RATE;
	raw_target.velocity.x = x;
	raw_target.velocity.y = y;
	raw_target.velocity.z = z;
	raw_target.yaw_rate = yaw_rate * 0.01745329;
	set_raw_pub.publish(raw_target);
	return 0;
}

int set_angular_rate(double yaw_rate) //FLU rad/s , must set continously, or the vehicle stops after a few seconds.(failsafe feature) used for adjusting yaw without setting others.
{
	mavros_msgs::PositionTarget raw_target;
	raw_target.coordinate_frame = PositionTarget::FRAME_BODY_OFFSET_NED;
	raw_target.type_mask = PositionTarget::IGNORE_VX | PositionTarget::IGNORE_VY | PositionTarget::IGNORE_VZ | PositionTarget::IGNORE_AFX | PositionTarget::IGNORE_AFY | PositionTarget::IGNORE_AFZ | PositionTarget::IGNORE_YAW; //yaw_rate must be used with pose or vel.
	raw_target.position.x = 0;
	raw_target.position.y = 0;
	raw_target.position.z = 0;
	raw_target.yaw_rate = yaw_rate * 0.01745329;
	set_raw_pub.publish(raw_target);
	return 0;
}

/*²»ÔÊÐí²»¶Ïµ÷ÓÃ£¬Ö»ÄÜµ÷ÓÃÒ»´ÎºóµÈÍê³ÉÔÙµ÷ÓÃ¡£*///  ×ªÏà¶ÔµÄYAW
int set_pose_body(double x, double y, double z, double yaw) //flu meters rad. yaw = 0 when not used. x=y=z=0 when not used.
{
	mavros_msgs::PositionTarget raw_target;
	raw_target.coordinate_frame = PositionTarget::FRAME_BODY_OFFSET_NED;
	raw_target.type_mask = PositionTarget::IGNORE_VX | PositionTarget::IGNORE_VY | PositionTarget::IGNORE_VZ | PositionTarget::IGNORE_AFX | PositionTarget::IGNORE_AFY | PositionTarget::IGNORE_AFZ | PositionTarget::IGNORE_YAW_RATE;
	if (fabs(yaw) < 1e-6)
		raw_target.type_mask |= PositionTarget::IGNORE_YAW;
	raw_target.position.x = x;
	raw_target.position.y = y;
	raw_target.position.z = z;
	raw_target.yaw = yaw*0.01745329;
	set_raw_pub.publish(raw_target);
	return 0;
}

int set_break()
{
	mavros_msgs::PositionTarget raw_target;
	raw_target.coordinate_frame = PositionTarget::FRAME_BODY_OFFSET_NED;
	raw_target.type_mask = PositionTarget::IGNORE_VX | PositionTarget::IGNORE_VY | PositionTarget::IGNORE_VZ | PositionTarget::IGNORE_AFX | PositionTarget::IGNORE_AFY | PositionTarget::IGNORE_AFZ | PositionTarget::IGNORE_YAW_RATE;
	raw_target.position.x = 0;
	raw_target.position.y = 0;
	raw_target.position.z = 0;
	raw_target.yaw = 0;
	set_raw_pub.publish(raw_target);
	return 0;
}  //ÉèÖÃÉ²³µ

void set_camera_down_mode(int mode) //ÉèÖÃÏà»ú
{
	std_msgs::Int8 mode_pub_data;
	mode_pub_data.data = mode;
	mode_pub.publish(mode_pub_data);
}

void camera_front_capture(void)  //ÉèÖÃÏà»ú
{
	std_msgs::Int8 pub_data;
	pub_data.data = 2;
	capture_pub.publish(pub_data);
}

template <typename T>
T bound(T const& a, T const& limit)
{
	if (a < -limit)
		return -limit;
	else if (a > limit)
		return limit;
	else
		return a;
}
enum Position_ControlMode
{
	//¿ØÖÆÆ÷Î´´ò¿ª
	Position_ControlMode_Null = 255,

	//ÆÕÍ¨Ä£Ê½	
	//Position_ControlMode_VelocityTrack = 16 ,	//ËÙ¶È¿ØÖÆ¸ú×ÙÄ£Ê½
	Position_ControlMode_Position = 12,	//Î»ÖÃËø¶¨Ä£Ê½
	Position_ControlMode_Velocity = 11,	//ËÙ¶È¿ØÖÆÄ£Ê½
	Position_ControlMode_Locking = 10,	//É²³µºóËøÎ»ÖÃ

	//2D×Ô¶¯Ä£Ê½
	Position_ControlMode_Takeoff = 20,	//Æð·ÉÄ£Ê½
	Position_ControlMode_RouteLine = 22,	//Ñ²ÏßÄ£Ê½

	//3D×Ô¶¯Ä£Ê½
	Position_ControlMode_RouteLine3D = 52,	//Ñ²ÏßÄ£Ê½
};

int main(int argc, char** argv)
{
	local_point waypts[] = {
		{1, 0, 1.0, 0},
		{1, 1, 1.0, 0},
		{0, 1, 1.0, 0},
		{0, 0, 1.0, 0},
		//		{0, 0, 1.0, 0},
		//		{0, 0, 1.0, 0},		
		//		{0, 2.5, 1.4, -3.1415926/2.0}
		//		{0, 0, 1, 0},
		//		{0, 0, 0.5, 0}
	};
	queue<local_point> waypoints;
	for (int i = 0; i < sizeof(waypts) / sizeof(local_point); i++)
	{
		waypoints.push(waypts[i]);
	}

	ros::init(argc, argv, "drone_node");
	ros::NodeHandle nh;
	// the setpoint publishing rate MUST be faster than 2Hz
	ros::Rate rate(20.0);
	ros::Subscriber		state_sub			=	nh.subscribe<mavros_msgs::State>			("/mavros/state", 10, state_cb);
						set_raw_pub			=	nh.advertise<mavros_msgs::PositionTarget>	("/mavros/setpoint_raw/local", 10);
						local_pos_pub		=	nh.advertise<geometry_msgs::PoseStamped>	("/mavros/setpoint_position/local", 10);
	/*****************************camera subs and pubs*****************************/
    ros::Subscriber		point_sub			=	nh.subscribe<geometry_msgs::Point>			("/camera_down/pose", 10, point_cb);
						mode_pub			=	nh.advertise<std_msgs::Int8>				("/camera_down/mode", 10);
	// ros::Subscriber	camera_front_sub	=	nh.subscribe<geometry_msgs::Point>			("/camera_front/vertex", 10, camera_front_cb);
	//					capture_pub			=	nh.advertise<std_msgs::Int8>				("/camera_front/capture", 10);
	// ros::Publisher	line_seq_pub		=	nh.advertise<std_msgs::UInt8>				("/which_line", 10);
	ros::Publisher		led_pub				=	nh.advertise<std_msgs::Bool>				("/led", 10);
	/*****************************camera subs and pubs end*************************/

	/*****************************camera of zhx subs and pubs***********************************/
	ros::Publisher		mode2cam			=	nh.advertise<std_msgs::Byte>				("/my_change_mode", 10);
	ros::Subscriber		cam_sub				=	nh.subscribe<geometry_msgs::PoseStamped>	("/mydrone", 10, drone_callback);//opencv
	ros::Subscriber		cam_sub2			=	nh.subscribe<std_msgs::Byte>				("/cvtomp", 10, cvtomp_callback);
	ros::Subscriber     t265_sub = nh.subscribe("/camera/odom/sample", 10, t265_callback);
	/*****************************camera of zhx subs and pubs end***********************************/
	ros::Subscriber		currentPos			=	nh.subscribe<geometry_msgs::PoseStamped>	("/mavros/local_position/pose", 10, pose_cb);
//	ros::Subscriber		front_dis_sub		=	nh.subscribe<sensor_msgs::Range>			("/mavros/distance_sensor/rangefinder_pub", 10, front_dis_cb);
	tf2_ros::TransformListener tfListener(tfBuffer);
	//	PD_Controller Point_Controller(Point_P_x, Point_D_x, Point_P_y, Point_D_y);
	mptocv.data = 1;

		// allow the subscribers to initialize
	
	ROS_INFO("INITILIZING...");
        
//ros::Duration(2.0).sleep();

//for(int ii=0;ii<3;ii++)
//{
//	mptocv.data = 7;
//	mode2cam.publish(mptocv);
//      ros::Duration(0.01).sleep();
//}
	
	for (int i = 0; i < 100; i++)
	{	
		ros::spinOnce();
		ros::Duration(0.01).sleep();
	}

	while (current_state.mode != "OFFBOARD") //wait for remote command
	{	
		ros::spinOnce();
		ros::Duration(0.01).sleep();
	}

	for (int i = 0; i < 200; i++)
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
	//	get_Altitude_ControlMode(nh);
	//	get_Position_ControlMode(nh);
		//arm
	arm_drone(nh);
ROS_INFO("xxx");
	while (ros::ok() && !current_state.armed)
	{
ROS_INFO("yyy:%d   %d",ros::ok(),current_state.armed);
		ros::spinOnce();
ROS_INFO("zzz");
		rate.sleep();
	}
	send_tf_snapshot_takeoff();//¼ÇÂ¼Æð·ÉÎ»ÖÃ
       ROS_INFO("aaa");
	//request takeoff
	takeoff(nh, 1.0); //meters
ROS_INFO("bbb");
	ros::spinOnce();
ROS_INFO("ccc");
	for (int i = 0; i < 16; i++)
	{
ROS_INFO("ddd");
		ros::spinOnce();
		ros::Duration(0.4).sleep();
	}
ROS_INFO("eee");
	rate.sleep();
	ros::spinOnce();
for(int ii=0;ii<3;ii++)
{
	mptocv.data = 1;
	mode2cam.publish(mptocv);
        ros::Duration(0.01).sleep();
}
	for (int i = 0; i < 10; i++)
	{	
		ros::spinOnce();
		ros::Duration(0.05).sleep();
	}

	
	for (int i = 0; i < 10000; i++)//ËÙ¶ÈÐèÒªÒ»Ö±·¢£¬1sÉÏËø
	{
		ros::spinOnce();
		ros::Duration(0.05).sleep();
		if(cvstate==2)
		{
ROS_INFO("RESIVED:::::");
			set_break();
			break;
		}
		set_speed_body(ctrl_cv.pose.position.x, ctrl_cv.pose.position.y, ctrl_cv.pose.position.z, ctrl_cv.pose.orientation.w);
		ros::spinOnce();
		ros::Duration(0.05).sleep();
		//ROS_INFO("has reached %.2f",posx_t265_local);
	}

	set_pose_body(0,-0.7,0.6,0);
	for (int i = 0; i < 14; i++)
	{
		ros::spinOnce();
		ros::Duration(0.5).sleep(); //ËÙ¶È²»ÄÜÌ«¿ì
	}
for (int i = 0; i < 5; i++)
	{
	mptocv.data = 2;
	mode2cam.publish(mptocv);
ros::Duration(0.02).sleep();
}
	for (int i = 0; i < 10; i++)
	{	
		ros::spinOnce();
		ros::Duration(0.05).sleep();
	}

	for (int i = 0; i < 10000; i++)//ËÙ¶ÈÐèÒªÒ»Ö±·¢£¬1sÉÏËø
	{
		ros::spinOnce();
		ros::Duration(0.05).sleep();
		if(cvstate==3)
		{
			set_break();
			break;
		}
		set_speed_body(ctrl_cv.pose.position.x, ctrl_cv.pose.position.y, ctrl_cv.pose.position.z, ctrl_cv.pose.orientation.w);
		ros::spinOnce();
		ros::Duration(0.05).sleep();
		ROS_INFO("has reached %.2f",posx_t265_local);
	}

	land(nh);
	//for (int i = 0; i < 100; i++)//ËÙ¶ÈÐèÒªÒ»Ö±·¢£¬1sÉÏËø
	//{
	//	set_speed_body(0.2, 0, 0, 0);
	//	ros::spinOnce();
	//	ros::Duration(0.05).sleep();
	//}
	//set_break();//É²³µ
	//for (int i = 0; i < 300; i++)
	//{
	//	ros::spinOnce();
	//	ros::Duration(0.01).sleep();
	//}

	//set_pose_body(-1,1,0,0);
	//for (int i = 0; i < 14; i++)
	//{
	//	ros::spinOnce();
	//	ros::Duration(0.5).sleep(); //ËÙ¶È²»ÄÜÌ«¿ì
	//}


	//for (int i = 0; i < 100; i++)
	//{
	//	set_speed_enu(0, -0.2, 0, 0);
	//	ros::spinOnce();
	//	ros::Duration(0.05).sleep();
	//}
	//set_break();
	//for (int i = 0; i < 300; i++)
	//{
	//	ros::spinOnce();
	//	ros::Duration(0.01).sleep();
	//}


	//set_pose_body(0, 0, 0, 90);  //Ðý×ª 90 du
	//for (int i = 0; i < 1000; i++)//10Ãë
	//{
	//	ros::spinOnce();
	//	ros::Duration(0.01).sleep();
	//}
	//set_break();

	//for (int i = 0; i < 300; i++)//3Ãë
	//{
	//	ros::spinOnce();
	//	ros::Duration(0.01).sleep();
	//}


	//for (int i = 0; i < 100; i++)
	//{
	//	set_angular_rate( -18 );
	//	ros::spinOnce();
	//	ros::Duration(0.05).sleep();
	//}
	//set_break();


	//for (int i = 0; i < 500; i++)
	//{
	//	ros::spinOnce();
	//	ros::Duration(0.01).sleep();
	//}
	////land
	//land(nh);

	while (ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}

