#include<opencv2/opencv.hpp>
#include <iostream>
#include <fcntl.h>
#include <linux/videodev2.h>
#include<sys/ioctl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <error.h>
#include <string.h>
#include <opencv2/cudaimgproc.hpp>
#include <ros/ros.h>
#include <ros/duration.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <vector>

using namespace cv;
using namespace std;
#define cam_down (get_cam_num(0))
#define cam_front (get_cam_num(1))
Mat srcImage;
Mat grayImage;
//Canny边缘检测相关变量
int g_canny_hough = 110;
int g_houghcircles1 = 14;
int g_houghcircles2 = 66;
int g_houghcircles3 = 35;
int iLowH = 155;
int iHighH = 179;
int iLowS = 50;
int iHighS = 245;
int iLowV = 50;
int iHighV = 245;
int g_cannyLowThreshold = 20;
geometry_msgs::Point target_position;

ros::Subscriber mode_sub;
ros::Publisher pos_pub;
int mode=1;
void huofucircles();//定点（霍夫圆检测）
void color_distinguish();//颜色识别

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

void mode_cb(const std_msgs::Int8::ConstPtr& msg)
{
  mode = msg->data;
}

int main(int argc, char** argv) 
{
	VideoCapture capture(cam_down,CAP_V4L2);

	if (!capture.isOpened())//如果视频不能正常打开则返回
	{
		cout << "摄像头打开失败！" << endl;
		return -1;
	}
	ros::init(argc, argv, "camera_down_node");
	ros::NodeHandle nh;
	pos_pub = nh.advertise<geometry_msgs::Point>("/camera_down/pose", 10);
	mode_sub = nh.subscribe<std_msgs::Int8>("/camera_down/mode", 10, mode_cb);
	while (ros::ok())
	{
		double fps,t=0;
		
		t = (double)getTickCount();
		capture >> srcImage;//等价于 capture.read(frame);
		if (srcImage.empty())//如果某帧为空则退出循环
		{
			cout << "摄像头断开！" << endl;
			break;
		}
		
		namedWindow("video");
		createTrackbar("mode：", "video", &mode, 5);
		if(mode==1)
		{
			huofucircles();
		}
		if(mode==2)
		{
			color_distinguish();			
		}
		pos_pub.publish(target_position);
		ros::spinOnce();
		waitKey(1);//每帧延时 1 毫秒，如果不延时，图像将无法显示
		t = ((double)getTickCount() - t) / getTickFrequency();
		fps = 1.0 / t;
		cout<<"fps:"<<fps<<endl;
	}

	capture.release();//释放资源
	return 0;
}

//定点（霍夫圆检测）
void huofucircles()
{
	//改变图像大小
	resize(srcImage, srcImage, Size(320, 240), 0, 0, INTER_AREA);

	// 创建显示窗口
	namedWindow("Canny", WINDOW_AUTOSIZE);
	// 创建滑动条
	createTrackbar("canny", "Canny", &g_canny_hough, 120);
	createTrackbar("最小半径：", "Canny", &g_houghcircles1, 300);
	createTrackbar("最大半径", "Canny", &g_houghcircles2, 1000);
	createTrackbar("累加器阈值", "Canny", &g_houghcircles3, 300);

	// 将原图像转换为灰度图像
	Mat grayImage, midImage;
	cvtColor(srcImage, grayImage, COLOR_BGR2GRAY);
	Canny(grayImage, midImage, g_canny_hough, 3 * g_canny_hough, 3);//进行一次canny边缘检测

	//进行霍夫圆变换
	vector<Vec3f> circles;
	HoughCircles(midImage, circles, HOUGH_GRADIENT, 1.5, 100, 100, g_houghcircles3, g_houghcircles1, g_houghcircles2);

	int k = 0, send_x = 0, send_y = 0;
	//依次在图中绘制出圆
	for (size_t i = 0; i < circles.size(); i++)
	{
		//参数定义
		Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
		int radius = cvRound(circles[i][2]);
		if (cvRound(circles[k][2])<cvRound(circles[i][2]))
		{
			k = i;
		}
		//绘制圆心
		circle(srcImage, center, 3, Scalar(0, 255, 0), -1, 8, 0);
		//绘制圆轮廓
		circle(srcImage, center, radius, Scalar(155, 50, 255), 2, 8, 0);
		//printf("%d,%d\n",center.x,center.y);
	}
	if (circles.size() != 0)
	{
		send_x = cvRound(circles[k][0]);
		send_y = cvRound(circles[k][1]);
		if (send_x != 0 && send_y != 0)
		{
			//发送数据
			cout << 120 - send_y << "," << 160 - send_x << endl;
			target_position.x=srcImage.cols/2 - send_x;
			target_position.y=srcImage.rows/2 - send_y;
			target_position.z=0;
		}
	}

	imshow("Canny", midImage);
	imshow("video", srcImage);
}

//颜色识别
void color_distinguish()
{
	resize(srcImage, srcImage, Size(320, 240), 0, 0, INTER_AREA);
	
	//创建显示窗口和滑动条
	namedWindow("Color");
	createTrackbar("LowH", "Color", &iLowH, 179); //Hue (0 - 179)
	createTrackbar("HighH", "Color", &iHighH, 179);
	createTrackbar("LowS", "Color", &iLowS, 255); //Saturation (0 - 255)
	createTrackbar("HighS", "Color", &iHighS, 255);
	createTrackbar("LowV", "Color", &iLowV, 255); //Value (0 - 255)
	createTrackbar("HighV", "Color", &iHighV, 255);
	
	Mat imgHSV;
	Mat imgThresholded;//输出的检测后的二值图
	Mat imgThresholded1;
	Mat imgThresholded2;
	Mat imgCanny;//canny算子轮廓检测的图
	Mat element;
	vector<Mat> hsvSplit;
	vector<vector<Point>> contours;
	//颜色空间转换
	cvtColor(srcImage, imgHSV, COLOR_BGR2HSV);
	//颜色检测
	//inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded);
	inRange(imgHSV, Scalar(0, 50, 50), Scalar(20, 245, 245), imgThresholded1);
	inRange(imgHSV, Scalar(150, 50, 50), Scalar(179, 245, 245), imgThresholded2);
	imgThresholded=imgThresholded1+imgThresholded2;
	//先开操作后闭操作去除噪点
	element = getStructuringElement(MORPH_RECT, Size(3, 3));
	morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);
	morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, element);

	//寻找轮廓
	Canny(imgThresholded, imgCanny, g_cannyLowThreshold, g_cannyLowThreshold * 3);
	findContours(imgCanny, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);
	//drawContours(imgCanny,contours,-1,Scalar(122,135,255),4);

	//寻找最大轮廓
	int min_length=20;
	int min_k = -1;
	if (contours.size()>0)
	{
		bool first = true;
		int min_distance;
		for (int i = 0; i<contours.size(); i++)
		{
			if (arcLength(contours[i], 0)>min_length)
			{
				//计算最大轮廓中心点坐标
				Point2f vertices[4];
				Point2f center_box;
				RotatedRect box = minAreaRect(contours[i]);//寻找最小包围矩形
				box.points(vertices);
				//计算中心点坐标
				center_box.x = (vertices[0].x + vertices[2].x) / 2.0;
				center_box.y = (vertices[0].y + vertices[2].y) / 2.0;
				if(first)
				{
					first=false;
					min_distance=abs(srcImage.cols/2-center_box.x)+abs(srcImage.rows/2 - center_box.y);
					min_k=i;
				}
				else if(abs(srcImage.cols/2-center_box.x)+abs(srcImage.rows/2 - center_box.y)<min_distance)
				{
					min_distance=abs(srcImage.cols/2-center_box.x)+abs(srcImage.rows/2 - center_box.y);
					min_k=i;
				}
			}
		}
		if(min_k!=-1)
		{
			//绘制最大轮廓
			drawContours(srcImage, contours, min_k, Scalar(122, 135, 255), 4);
			//计算最大轮廓中心点坐标
			Point2f vertices[4];
			Point2f center_box;
			RotatedRect box = minAreaRect(contours[min_k]);//寻找最小包围矩形
			box.points(vertices);
			//计算中心点坐标
			center_box.x = (vertices[0].x + vertices[2].x) / 2.0;
			center_box.y = (vertices[0].y + vertices[2].y) / 2.0;
			circle(srcImage, center_box, 3, Scalar(0, 255, 0), -1, 8, 0);
			cout << srcImage.cols/2 - center_box.x << "," << srcImage.rows/2 - center_box.y << endl;
			target_position.x=srcImage.cols/2 - center_box.x;
			target_position.y=srcImage.rows/2 - center_box.y;
			target_position.z=0;
		}
	}
	if(min_k==-1)
	{
		target_position.x=0;
		target_position.y=0;
		target_position.z=0;
	}

	imshow("Color", imgThresholded);
	//imshow("Canny",imgCanny);
	imshow("video", srcImage);
}