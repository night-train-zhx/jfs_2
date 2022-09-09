
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandHome.h>
#include <mavros_msgs/PositionTarget.h>
#include <sensor_msgs/Range.h>
#include <iostream>
#include <stdio.h>
//#include <wiringPi.h>
#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Byte.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
#include <sys/time.h>
#include <pthread.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <stdlib.h>
#include <math.h>

using namespace cv;
using namespace std;
//pthread_mutex_t mutex_cam_data;
pthread_mutex_t mutex_dptosd;
/// Global variables

double t = 0,fps=0,timecount=0;
char dpstate=0;
char template_dptosd=0;
int ttt=0,summ=0,ave=0,i=0,j=0,yxh=0,shu=0,midline=0,piancha=0;
float y_ctrl=0;
short int COL[640]={0},ROW[480]={0};
Mat frame,img,src,sec_2,gray;
Mat1b mask;
unsigned char bz_start=0,bz_send=1;

float constrain_float(float x,float bound)
{
	if( x > bound )
		return bound;
	else if( x < -bound )
		return -bound;
	else return x;
}

int jdz_i(int jj)
{
	if(jj<0) jj=jj*(-1);
	return jj;
}

void dptosd_callback(const std_msgs::Byte& msg)
{
	pthread_mutex_lock(&mutex_dptosd);
	template_dptosd=msg.data;
	pthread_mutex_unlock(&mutex_dptosd);
	ROS_INFO("video:%d" ,msg.data );
	//ROS_INFO("video:%.2f , %.2f", msg.pose.position.x, msg.pose.position.z);
}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"star_detection");
    ros::NodeHandle node_obj;
	pthread_mutex_init (&mutex_dptosd, NULL);

    ros::Publisher pub_sdtodp=node_obj.advertise<geometry_msgs::PoseStamped>("/sdtodp", 10);
    ros::Subscriber sub_dptosd = node_obj.subscribe("/dptosd",10,dptosd_callback);
    ros::Rate rate(30);
    geometry_msgs::PoseStamped msg_velocity;

    VideoCapture capture("/dev/v4l/by-id/usb-Generic_Rmoncam_A2_720P_200901010001-video-index0");
    if (!capture.isOpened())
    {  printf("err");
        return -1;}

while(1)
{  
    ros::spinOnce();
    pthread_mutex_lock(&mutex_dptosd);
    dpstate = template_dptosd;
    pthread_mutex_unlock(&mutex_dptosd);
    t = (double)cv::getTickCount();
ROS_INFO("dpstate:%d   bz_send:%d",dpstate,bz_send);
/*
*/
    if((dpstate==1)&&(bz_send==1))
    {
        printf("sd_opened");
        capture>>frame;
//ROS_INFO("COL:%d  %d",frame.cols,frame.rows);
   Rect rec = Rect(100,20,440,440);
        src=frame(rec);

        cvtColor(src, gray, COLOR_BGR2GRAY);
         GaussianBlur(gray, gray, Size(9,9),3,3);
   Mat show = src.clone();
        
        vector<Vec3f>circles;
        float sum_x=0,sum_y=0;
        HoughCircles(gray, circles, HOUGH_GRADIENT, 1, 20, 100, 63, 0, 0);
        //Mat show = src.clone();
        if(circles.size()>0)
        {
            for (int i = 0; i < circles.size(); i++)
            {
                sum_x+=circles[i][0];
                sum_y+=circles[i][1];
                circle(show, Point(circles[i][0], circles[i][1]), circles[i][2], Scalar(0, 0, 255), 2);
circle(show, Point(circles[i][0], circles[i][1]), 4, Scalar(0, 255, 255), -1);
            }

        
            float ave_x=sum_x/(float)(circles.size());
            float ave_y=sum_y/(float)(circles.size());
            sum_x=0;sum_y=0;
            float piancha_x=280-ave_y;
            float piancha_y=(gray.cols/2)-ave_x;
            float x_ctrl=constrain_float(piancha_x*0.022,0.2);
            float y_ctrl=constrain_float(piancha_y*0.022,0.2);
            float z_ctrl=5;
            //printf("%.2f\n",y_ctrl);
            if((jdz_i((int)(ave_x)-gray.cols/2)<(gray.cols/4))&&(jdz_i((int)(ave_y)-gray.rows/2)<(gray.rows/4)))
            {
                if(bz_start==0)
                {
                    bz_start=1;
                    timecount=0;
                }
                if(bz_start==1)
                {
                    if(timecount>1.3)
                    {
                        bz_start=0;
                        timecount=0;
                        x_ctrl=0;
                        y_ctrl=0;
                        z_ctrl=6;
                        bz_send=0;
                    }
                }
            }
            else
            {
                bz_start=0;
                timecount=0;
            }

            msg_velocity.pose.position.x=x_ctrl;
            msg_velocity.pose.position.y=y_ctrl;
            msg_velocity.pose.position.z=z_ctrl;
            pub_sdtodp.publish(msg_velocity);
printf("have sent:xxx:%.3f,,yyy:%.3f,,,zzz%.3f",x_ctrl,y_ctrl,z_ctrl);
        }
imshow("circle", show);
    }
//imshow("circle", show);
    waitKey(1);
    rate.sleep();
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    fps = 1.0 / t;
    timecount+=t;
    //printf("FPS:%d//////miaoshu:%.4f\n",(int)fps,t);
}
  return(0);
}

