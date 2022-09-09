#include <opencv2/opencv.hpp>
#include <iostream>
#include <fcntl.h>
#include <string>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <error.h>
#include <string.h>
#include <opencv2/cudaimgproc.hpp>
#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
using namespace cv;
using namespace std;
#define cam_down (get_cam_num(0))
#define cam_front (get_cam_num(1))
#define PI 3.1415927

VideoCapture capture;
Mat srcImage;
//Canny边缘检测相关变量
int g_cannyLowThreshold = 20;
int g_canny_line = 75;
int g_houghlines1 = 80;
int g_houghlines2 = 50;
int g_houghlines3 = 10;
int thresh = 100;
int which_line = 0;

void huofulines();//巡线（霍夫线变换）
void color_distinguish();//颜色识别
int get_cam_num(int cam_id);//id=0:down id=1:front
ros::Publisher publisher_vertex;
ros::Publisher publisher_led;

void change_line(const std_msgs::UInt8& msg)
{
	which_line=msg.data;
}

int main(int argc, char** argv) 
{
	ros::init(argc, argv, "camera_front");
	ros::NodeHandle node_obj;
	publisher_vertex = node_obj.advertise<geometry_msgs::Point>("/camera_front/vertex", 10);
	publisher_led = node_obj.advertise<std_msgs::Bool>("/led", 10);
	ros::Subscriber subscriber_line = node_obj.subscribe("/which_line",10,change_line);//切换模式的话题
	ros::Rate rate(30.0);
	capture.open(cam_front,CAP_V4L2);//打开摄像头
	if (!capture.isOpened())//如果视频不能正常打开则返回
	{
		cout << "摄像头打开失败！" << endl;
		return 1;
	}

	while (ros::ok())
	{
		capture >> srcImage;//等价于 capture.read(frame);
		if (srcImage.empty())//如果某帧为空则退出循环
		{
			cout << "摄像头断开！" << endl;
			break;
		}
		huofulines();//巡线（霍夫线变换）
		capture >> srcImage;
		color_distinguish();
		ros::spinOnce();
		rate.sleep();

		waitKey(1);//每帧延时 1 毫秒，如果不延时，图像将无法显示
	}

	capture.release();//释放资源
	return 0;
}

int get_cam_num(int cam_id)//id=0:down id=1:front
{
	struct v4l2_capability cap;
	bool exist_0=true;
	bool exist_1=true;
	int num[2]={-1,-1};
	int fd=open("/dev/video0",O_RDWR);
	if(fd<0)exist_0=false;
	else
	{
		int ret = ioctl(fd, VIDIOC_QUERYCAP, &cap);
		if (ret < 0) 
		{
			perror("ioctl querycap");
			return -1;
		}
		num[0]=cap.bus_info[strlen((const char *)cap.bus_info)-1]-'3';
		//cout<<cap.bus_info<<endl;
		close(fd);
	}
	fd=open("/dev/video1",O_RDWR);
	if(fd<0)exist_1=false;
	else
	{
		int ret = ioctl(fd, VIDIOC_QUERYCAP, &cap);
		if (ret < 0) 
		{
			perror("ioctl querycap");
			return -1;
		}
		//cout<<cap.bus_info<<endl;
		num[1]=cap.bus_info[strlen((const char *)cap.bus_info)-1]-'3';
		close(fd);
	}
	int id[2]={-1,-1};
	if((!exist_0)&&(!exist_1)) return -1;
	else if(exist_0&&(!exist_1))return 0;
	else if((!exist_0)&&exist_1)return 1;
	if(num[0]==1)
	{
		id[1]=0;
		id[0]=1;
	}
	else
	{
		id[0]=0;
		id[1]=1;
	}
	return id[cam_id];
}


//巡线（霍夫线变换）
void huofulines()
{
	//resize(srcImage,srcImage,Size(320,240));
	//创建窗口和滑动条
	namedWindow("line");
	createTrackbar("canny", "Canny", &g_canny_line, 100);
	createTrackbar("累加平面的阈值参数（巡线）", "line", &g_houghlines1, 300);
	createTrackbar("最短线段长度", "line", &g_houghlines2, 250);
	createTrackbar("允许连接的最大距离", "line", &g_houghlines3, 250);
	Mat grayImage;
	Mat midImage;
	Mat element;
	cvtColor(srcImage, grayImage, COLOR_RGB2GRAY);
	//形态学滤波操作
	element = getStructuringElement(MORPH_RECT, Size(9, 9));
	morphologyEx(grayImage, grayImage, MORPH_CLOSE, element);
	morphologyEx(grayImage, grayImage, MORPH_OPEN, element);
	GaussianBlur(grayImage, grayImage, Size(3, 3), 0, 0);
	Canny(grayImage, midImage, g_canny_line, g_canny_line * 3, 3);
	//进行霍夫线变换
	vector<Vec4i> lines;//定义一个矢量结构lines用于存放得到的线段矢量集合
	HoughLinesP(midImage, lines, 1, CV_PI / 180, g_houghlines1, g_houghlines2, g_houghlines3);
	vector<Point3f> exist_line;//xyz分别存储斜率、截距、数量
	//将竖直方向的线存入数组中
	for (size_t i = 0; i < lines.size(); i++)
	{
		Vec4i l = lines[i];
		line(srcImage, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(186, 88, 255), 1, LINE_AA);
		//计算斜率和横截距
		float ratio = (l[1] - l[3])*1.0 / (l[0] - l[2]);
		float intercept = l[1] - ratio*l[0];
		//寻找有没有重复的线
		bool is_exist=false;
		for(int i=0;i<exist_line.size();i++)
		{
			if(abs(atan(exist_line[i].x)-atan(ratio))<12/180.0*1.57)
			{
				is_exist=true;
				exist_line[i].x = (exist_line[i].x*exist_line[i].z+ratio)/(exist_line[i].z+1);
				exist_line[i].y = (exist_line[i].y*exist_line[i].z+intercept)/(exist_line[i].z+1);
				exist_line[i].z++;
				break;
			}
		}
		if(is_exist == false)
		{
			exist_line.push_back(Point3f(ratio,intercept,1));
		}
	}
	for(int i=0;i<exist_line.size();i++)
	{
		Point draw1,draw2;
		if(exist_line[i].x>-1 && exist_line[i].x<1)
		{
			draw1=Point(0,exist_line[i].y);
			draw2=Point(640,exist_line[i].x*640+exist_line[i].y);
		}
		else
		{
			draw1=Point(-exist_line[i].y/exist_line[i].x,0);
			draw2=Point((480-exist_line[i].y)/exist_line[i].x,480);
		}
		line(srcImage, draw1, draw2, Scalar(186, 88, 255), 3, LINE_AA);
	}
	if(exist_line.size()>=2)
	{
		//算出交点
		Point intersect;
		intersect.x=(exist_line[0].y-exist_line[1].y)*1.0/(exist_line[1].x-exist_line[0].x);
		intersect.y=exist_line[0].x*intersect.x+exist_line[0].y;
		circle(srcImage, intersect, 3, Scalar(0, 255, 0), -1, 8, 0);
		geometry_msgs::Point msg; //发送的消息
		msg.x = 320-intersect.x;
		msg.y = 240-intersect.y;
		msg.z = 0;
		publisher_vertex.publish(msg);
	}
	else
	{
		geometry_msgs::Point msg; //发送的消息
		msg.x = 0;
		msg.y = 0;
		msg.z = 0;
		publisher_vertex.publish(msg);
	}
	imshow("Canny", midImage);
	imshow("line", srcImage);
	return;
}

//颜色识别
void color_distinguish()
{
	resize(srcImage, srcImage, Size(320, 240), 0, 0, INTER_AREA);
	
	Mat imgHSV;
	Mat imgThresholded1,imgThresholded2,imgThresholded;//输出的检测后的二值图
	Mat imgCanny;//canny算子轮廓检测的图
	Mat element;
	double length_max = 0;
	vector<Mat> hsvSplit;
	vector<vector<Point>> contours;
	//颜色空间转换
	cvtColor(srcImage, imgHSV, COLOR_BGR2HSV);
	//颜色检测
	inRange(imgHSV, Scalar(155, 50, 50), Scalar(179, 245, 245), imgThresholded1);
	inRange(imgHSV, Scalar(0, 50, 50), Scalar(20, 245, 245), imgThresholded2);
	imgThresholded=imgThresholded1+imgThresholded2;
	//先开操作后闭操作去除噪点
	element = getStructuringElement(MORPH_RECT, Size(3, 3));
	morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);
	morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, element);

	//寻找轮廓
	Canny(imgThresholded, imgCanny, g_cannyLowThreshold, g_cannyLowThreshold * 3);
	findContours(imgCanny, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);

	//寻找最大轮廓
	int i, k;
	if (contours.size()>0)
	{
		k = 0;
		for (i = 0; i<contours.size(); i++)
		{

			if (arcLength(contours[i], 0)>length_max)
			{
				length_max = arcLength(contours[i], 0);
				k = i;
			}
		}
		//绘制最大轮廓
		drawContours(srcImage, contours, k, Scalar(122, 135, 255), 1);
		//计算最大轮廓中心点坐标
		if(length_max>17)
		{
			drawContours(srcImage, contours, k, Scalar(122, 135, 255), 3);
			Point2f vertices[4];
			Point2f center_box;
			RotatedRect box = minAreaRect(contours[k]);//寻找最小包围矩形
			box.points(vertices);
			//计算中心点坐标
			center_box.x = (vertices[0].x + vertices[2].x) / 2.0;
			center_box.y = (vertices[0].y + vertices[2].y) / 2.0;
			circle(srcImage, center_box, 3, Scalar(0, 255, 0), -1, 8, 0);
			if(center_box.x>80 && center_box.x<240 && center_box.y>60 && center_box.x<180)
			{
				string name=to_string(which_line)+".jpg";
				if(which_line==0 || which_line==11)
				{
					std_msgs::Bool msg; //发送的消息
					msg.data=false;
					publisher_led.publish(msg);
				}
				else
				{
					imwrite(name,srcImage);
					std_msgs::Bool msg; //发送的消息
					msg.data=true;
					publisher_led.publish(msg);
				}
			}
		}
		else
		{
			std_msgs::Bool msg; //发送的消息
			msg.data=false;
			publisher_led.publish(msg);
		}
	}
	else
	{
		std_msgs::Bool msg; //发送的消息
		msg.data=false;
		publisher_led.publish(msg);
	}

	imshow("Color", imgThresholded);
	imshow("video", srcImage);
}