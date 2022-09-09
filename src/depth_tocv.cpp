
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
//#include <zbar.h>
#include <cv_bridge/cv_bridge.h>

#define narr(a) (int)(a*255/300)

using namespace std;
using namespace cv;

cv::Mat data;
cv::Mat data_d,data_tmp,fake_color;
Mat show_d,show_r,frame;
Mat imageSource;
Mat hsv;

double t = 0,fps,timecount=0;
int mpstate=0;
char template_mps=0;
unsigned char bz_start=0,bz_bar=0,bz_red=0,bz_max=0,bz_jiao=0,bz_bar2cer=0,bz_t265=0,bz_2to3=0;
float max_dep_last=0;
int max_dep=200,max_dep_0=300,max_dep_1=120,min_dep_1=20;
int count_ctrl=0,count_found=0,count_shot=0,number_shot=0,count_fsm4=0,count_2to3=0,count_ring=0;
float piancha_x=0,piancha_y=0,piancha_z=0,piancha_yaw=0;
float x_ctrl=0,y_ctrl=0,z_ctrl=0,yaw_ctrl=0;
int fsm_inner=1,fsm=-2;
int bz_found_land=0;
float bar_dep_first;

pthread_mutex_t mutex_data_d;
pthread_mutex_t mutex_data_r;
pthread_mutex_t mutex_mptocv;
pthread_mutex_t mutex_t265;
pthread_mutex_t mutex_sdtodp;
float posz_t265_start=0;
float posz_t265=0;
float posx_t265=0;
float posy_t265=0;
float ctrlx_land=0;
float ctrly_land=0;
float bz_land=0;
std_msgs::Byte cvtomp;
std_msgs::Byte msgtopy;

struct MY_cts
{
int area;
Point2i center;
float radius;
int index;
float angle;
vector<Point> ct;
};

float jdz_f(float jj)
{
    if(jj<0) jj=jj*(-1);
    return jj;
}

int jdz_i(int jj)
{
    if(jj<0) jj=jj*(-1);
    return jj;
}

float constrain_float(float x,float bound)
{
    if( x > bound )
        return bound;
    else if( x < -bound )
        return -bound;
    else return x;
}

int constrain(int x,int minn,int maxx)
{
    if( x > maxx )
        return maxx;
    else if( x < minn )
        return minn;
    else return x;
}

void tsf16_8(Mat &sixteen,Mat &eight,int minn,int maxx)
{//minn!=0   maxx!=minn
  int temp;
  float kk=(255.0/(float)(maxx-minn));
  float bb=(255.0/(float)(1-maxx/minn));
  for(int i=0;i<sixteen.rows;i++)
  {
    for(int j=0;j<sixteen.cols;j++)
    {
      temp=sixteen.at<uint16_t>(i,j);
      temp=(int)(kk*temp+bb);
      if(temp<0) temp=0;
      else if(temp>255) temp=255;
      eight.at<uchar>(i,j)=temp;
    }
  }
}

void select_depth(Mat &img,int minn,int maxx)
{
   //建立查找表
  unsigned char lookup[256];
  for(int i=0; i<256; i++)
  {
    if(i>=maxx)
      lookup[i]=0;
    else if(i<=minn)
      lookup[i]=0;
    else
      lookup[i]=255;
  }
  int rows=img.rows;
  int cols=img.cols*img.channels();
  for(int i=0; i<rows; i++)
  {
    uchar *p=img.ptr<uchar>(i);
    for(int j=0; j<cols;j++)
      p[j]=lookup[p[j]];
  }
}

float getlinelegth(int x1,int y1,int x2,int y2)
{
    float length;
    length=sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
    return length;
}

float getangle(int x1,int y1,int x2,int y2)
{
    float jiao=0;
    if(x1==x2)
    {
        jiao=90;
    }
    else
    {
        float xielv=(float)(y2-y1)/(float)(x2-x1);
        jiao=57.296*atan(xielv);
    }
    return jiao;
}

int get_cross(Point2i &cross,Point2f &aa,Point2f &bb,float jiao_1,float jiao_2)
{
    if(jdz_f(jiao_1-jiao_2)<=2)
        return 0;

    else
    {
        Point2f cross_f;
        jiao_1=jiao_1/57.296;
        jiao_2=jiao_2/57.296;
        cross_f.x=bb.x+(((bb.y-aa.y)*cos(jiao_1)-(bb.x-aa.x)*sin(jiao_1))/sin(jiao_1-jiao_2))*cos(jiao_2);
        cross_f.y=bb.y+(((bb.y-aa.y)*cos(jiao_1)-(bb.x-aa.x)*sin(jiao_1))/sin(jiao_1-jiao_2))*sin(jiao_2);
      //  printf("%.3f,,,%.3f\n",cross_f.x,cross_f.y);  //
        cross.x=(int)(cross_f.x);
        cross.y=(int)(cross_f.y);
     //   printf("%d,,,%d\n",cross.x,cross.y);  //
        return 1;
    }
}

int getcir(Point2i &aa,Point2i &bb,Point2i &cc,Point2i &center,float *radius)
{
    if(((aa.x==bb.x)&&(aa.y==bb.y))||((aa.x==cc.x)&&(aa.y==cc.y))||((bb.x==cc.x)&&(bb.y==cc.y)))
        return 0;
    else
    {
        float angle_1=0,angle_2=0;
        angle_1=getangle(aa.x,aa.y,bb.x,bb.y);
        angle_2=getangle(aa.x,aa.y,cc.x,cc.y);
     //   printf("%.3f,,,%.3f\n",angle_1,angle_2);  //
        if(jdz_f(angle_1-angle_2)<1.0)
            return 0;
        else
        {
            Point2f center_ab,center_ac;
            center_ab.x=(float)(aa.x+bb.x)/2;
            center_ab.y=(float)(aa.y+bb.y)/2;
            center_ac.x=(float)(aa.x+cc.x)/2;
            center_ac.y=(float)(aa.y+cc.y)/2;
      //      printf("%.3f,,,%.3f,,,%.3f,,,%.3f\n",center_ab.x,center_ab.y,center_ac.x,center_ac.y);  //
            float a_ab_vec=0,a_ac_vec=0;
            if(angle_1<=0)
                a_ab_vec=angle_1+90;
            else
                a_ab_vec=angle_1-90;
            if(angle_2<=0)
                a_ac_vec=angle_2+90;
            else
                a_ac_vec=angle_2-90;
     //       printf("%.3f,,,%.3f\n",a_ab_vec,a_ac_vec);  //
            Point2i center_temp;
            int iscross=get_cross(center_temp,center_ab,center_ac,a_ab_vec,a_ac_vec);
      //      printf("%d,,,%d\n",center_temp.x,center_temp.y);  //
            if(iscross==0)
                return 0;
            else
            {
                *radius=getlinelegth(center_temp.x,center_temp.y,aa.x,aa.y);
                center=center_temp;
                return 1;
            }
        }
    }
}


float get_center_dep(Mat &img,Mat &mask)
{
    float sum_dis=0;
    int count_dis=0;
    int rows=img.rows;
    int cols=img.cols*img.channels();
    for(int i=(rows*2/5); i<(rows*3/5); i++)
    {
        uint16_t *p=img.ptr<uint16_t>(i);
        uchar *m=mask.ptr<uchar>(i);
        for(int j=(cols*1/4); j<(cols*3/4);j++)
        {
            if(m[j]!=0)
            {
                sum_dis+=p[j];
                count_dis++;
            }
        }
    }
    float ave_dis=sum_dis/((float)(count_dis)*10.0);
    return ave_dis;
}

float get_bar_dep(Mat &img,Mat &mask)
{
    float sum_dis=0;
    int count_dis=0;
    int rows=img.rows;
    int cols=img.cols*img.channels();
    for(int i=0; i<rows; i++)
    {
        uint16_t *p=img.ptr<uint16_t>(i);
        uchar *m=mask.ptr<uchar>(i);
        for(int j=0; j<cols;j++)
        {
            if(m[j]!=0)  //add bitwise_not
            {
                sum_dis+=p[j];
                count_dis++;
            }
        }
    }
    float ave_dis=sum_dis/((float)(count_dis)*10.0);
    return ave_dis;
}

int countimage(Mat &img)
{
    int count_white=0;
    int rows=img.rows;
    int cols=img.cols*img.channels();
    for(int i=0; i<rows; i++)
    {
        uchar *m=img.ptr<uchar>(i);
        for(int j=0; j<cols;j++)
        {
            if(m[j]!=0)  //add bitwise_not
            {
                count_white++;
            }
        }
    }
    return count_white;
}

int find_red(Mat &img)
{
    Mat hsv;
    Mat1b mask;
    cvtColor(img, hsv, COLOR_BGR2HSV);//转为HSV
    Mat1b mask1,mask2;
    inRange(hsv,Scalar(0,120,50),Scalar(10,255,255),mask1);
    inRange(hsv,Scalar(170,120,50),Scalar(180,255,255),mask2);
    mask = mask1|mask2;
    Mat element = getStructuringElement(MORPH_RECT, Size(3,3));
    morphologyEx(mask, mask, MORPH_ERODE, element);
    int count_red=0;
    int rows=mask.rows;
    int cols=mask.cols*mask.channels();
    for(int i=(rows*1/4); i<(rows*3/4); i++)
    {
        uchar *p=mask.ptr<uchar>(i);
        for(int j=(cols*1/4); j<(cols*3/4);j++)
        {
            if(p[j]!=0)  //add bitwise_not
            {
                count_red++;
            }
        }
    }
    if(count_red>=50)
        return 1;
    else
        return 0;
}

float get_err_line(Mat &img)
{
    short int COL[640]={0},ROW[480]={0};
    int ttt=0,summ=0,ave=0,yxh=0,shu=0,midline=0;
    float piancha=0;
    for (int i = 0; i < img.cols; ++i)
    {
        for (int j = 0; j < img.rows; ++j)
        {
            if(img.at<uchar>(j,i)!=0)ttt++;
        }
        COL[i]=ttt;
        ttt=0;
        summ+=COL[i];
    }
    ave=summ/img.cols;
    summ=0;
    for (int i = 0; i < img.cols; ++i)
    {
        if(COL[i]>ave)
        {
            yxh+=i;
            shu++;
        }
    }
    if(yxh==0)
    {
        printf("nonono");
        midline=img.cols/2;
    }
    else
    {
        midline=yxh/shu;
        yxh=0;
        shu=0;
    }
printf("midline:%d\n",midline);
    piancha=(img.cols/2)-midline;
    return (float)piancha;
}

void camera_callback(const nav_msgs::Odometry& msg)
{
    pthread_mutex_lock(&mutex_t265);
    posx_t265=msg.pose.pose.position.x;
    posy_t265=msg.pose.pose.position.y;
if(bz_t265==0)
{
posz_t265_start=msg.pose.pose.position.z;
posz_t265=msg.pose.pose.position.z;
bz_t265=1;
}
else
{
    posz_t265=msg.pose.pose.position.z-posz_t265_start;//added by ujj
}
    //Height=msg.pose.pose.position.z-posz_t265_start;//ujj
    pthread_mutex_unlock(&mutex_t265);
//ujj	ROS_INFO("video:%.2f , %.2f ,%.2f", msg.pose.position.x, msg.pose.position.y,msg.pose.position.z);
//		ROS_INFO("video:%.2f , %.2f ,%.2f", posx_t265,posy_t265,posz_t265);
}

void sdtodp_Callback(const geometry_msgs::PoseStamped& msg)
{
    pthread_mutex_lock(&mutex_sdtodp);
    ctrlx_land=msg.pose.position.x;
    ctrly_land=msg.pose.position.y;
    bz_land=msg.pose.position.z;
    pthread_mutex_unlock(&mutex_sdtodp);
}

void mptocv_callback(const std_msgs::Byte& msg)
{
    pthread_mutex_lock(&mutex_mptocv);
    template_mps=msg.data;
    pthread_mutex_unlock(&mutex_mptocv);
    ROS_INFO("video:%d" ,msg.data );
    //ROS_INFO("video:%.2f , %.2f", msg.pose.position.x, msg.pose.position.z);
}

void ColorCallback(const sensor_msgs::ImageConstPtr& color_img) {
  //ROS_WARN("%s", color_img->encoding.c_str());
  pthread_mutex_lock(&mutex_data_r);
  data = cv_bridge::toCvShare(color_img, "bgr8")->image;
  //show_r=data.clone();
  pthread_mutex_unlock(&mutex_data_r);
  //std::cout << "image data: " << data.at<uchar>(240, 320) << std::endl;
 // cv::imshow("testcolor", data);
  //waitKey(1);
}

void DepthCallback(const sensor_msgs::ImageConstPtr& depth_img) {
  //ROS_WARN("%s", depth_img->encoding.c_str());
  pthread_mutex_lock(&mutex_data_d);
  data_d = cv_bridge::toCvCopy(depth_img,sensor_msgs::image_encodings::TYPE_16UC1)->image;
  //show_d=data_d.clone();
  pthread_mutex_unlock(&mutex_data_d);
  //std::cout << "image data: " << data_d.at<uint16_t>(240, 320) << std::endl;//表示获取图像坐标为240,320的深度值,单位是毫米
  Mat data8=Mat::zeros(data_d.size(),CV_8UC1);
  normalize(data_d,data_tmp,0,255,NORM_MINMAX);
  convertScaleAbs(data_tmp,data8);
  applyColorMap(data8,fake_color,cv::COLORMAP_JET);
  //cout<<"callback::::"<<show_d<<";"<<endl<<endl;
 // cv::imshow("testdepth", data_d);
 // cv::imshow("fakecolor", fake_color);
  //waitKey(1);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_depth");
    pthread_mutex_init(&mutex_data_r, NULL);
    pthread_mutex_init(&mutex_data_d, NULL);
    pthread_mutex_init(&mutex_t265, NULL);
    pthread_mutex_init(&mutex_mptocv, NULL);
    ros::NodeHandle node;
    ros::Publisher tomain_publisher=node.advertise<geometry_msgs::PoseStamped>("/mydrone", 10);//向飞控发送信息的话题
    ros::Publisher number_publisher2=node.advertise<std_msgs::Byte>("/cvtomp", 10);
    ros::Publisher pub_dptosd=node.advertise<std_msgs::Byte>("/dptosd", 10);
    ros::Subscriber number_subscriber3 = node.subscribe("/camera/odom/sample",10,camera_callback);
    ros::Subscriber number_subscriber1 = node.subscribe("/my_change_mode",10,mptocv_callback);
    ros::Subscriber sub_sdtodp = node.subscribe("/sdtodp", 10, sdtodp_Callback);
    ros::Subscriber depth_sub = node.subscribe<sensor_msgs::Image>("/d400/aligned_depth_to_color/image_raw", 2, DepthCallback);
    ros::Subscriber color_sub = node.subscribe<sensor_msgs::Image>("/d400/color/image_raw", 2, ColorCallback);
    ros::Publisher sendtopy = node.advertise<std_msgs::Byte>("/chatter", 10);
    ros::Rate rate(30);
    geometry_msgs::PoseStamped msg_velocity;
    
    while (1)
    {
        t = (double)cv::getTickCount();
        ros::spinOnce();
        pthread_mutex_lock(&mutex_data_d);
        show_d=data_d.clone();
        pthread_mutex_unlock(&mutex_data_d);
        pthread_mutex_lock(&mutex_data_r);
        show_r=data.clone();
        pthread_mutex_unlock(&mutex_data_r);
        pthread_mutex_lock(&mutex_t265);
        float posz_t265_local=posz_t265;
        pthread_mutex_unlock(&mutex_t265);
        pthread_mutex_lock(&mutex_mptocv);
        mpstate = template_mps;
        pthread_mutex_unlock(&mutex_mptocv);
        pthread_mutex_lock(&mutex_sdtodp);
        float ctrl_land_x = ctrlx_land;
        float ctrl_land_y = ctrly_land;
        bz_found_land = (int)(bz_land);
        pthread_mutex_unlock(&mutex_sdtodp);
        cout<<"fsm: "<<fsm<<endl;

        if((show_r.cols!=0)&&(show_d.cols!=0))
        {
            Rect rec = Rect((int)((show_r.cols-show_r.rows)/2),0,show_r.rows,show_r.rows);
            show_r=show_r(rec);
            resize(show_r, show_r, Size(300,300), 0,0, INTER_NEAREST);
             show_d=show_d(rec);
             resize(show_d, show_d, Size(300,300), 0,0, INTER_NEAREST);
            Mat mask_cir=Mat::zeros(Size(300,300), CV_8UC1);
            circle(mask_cir, Point(150,150), 149, Scalar(255), -1, 8);
            Mat cir;
            show_r.copyTo(cir,mask_cir);
            imageSource=show_r.clone();

            Mat gray=Mat::zeros(show_d.rows,show_d.cols,CV_8UC1);
            tsf16_8(show_d,gray,50,3050); //5cm--3.05m
        //    imshow("Source Image",show_r);
            Mat image,gry_2;

            if(mpstate==1)
            {
                if(fsm==-2)
                {
                    Mat binary=Mat::ones(show_d.rows,show_d.cols,CV_8UC1)*255;
                    select_depth(gray,narr(160),narr(270));
                    Mat element=getStructuringElement(MORPH_RECT,Size(6,6));
                    morphologyEx(gray,gray,MORPH_DILATE,element);
                    cir.copyTo(binary,gray);
                    Mat hsv;
                    int h_l=15,s_l=10,v_l=80,h_h=82,s_h=97,v_h=255;
                    cvtColor(binary, hsv, COLOR_BGR2HSV);//转为HSV
                    Mat1b mask;
                    inRange(hsv,Scalar(h_l,s_l,v_l),Scalar(h_h,s_h,v_h),mask);
                    Mat element1 = getStructuringElement(MORPH_RECT, Size(5,5));
                    morphologyEx(mask,mask, MORPH_ERODE, element1);
                    Mat element2 = getStructuringElement(MORPH_RECT, Size(4,4));
                    morphologyEx(mask,mask, MORPH_DILATE, element2);
                    Mat element3 = getStructuringElement(MORPH_RECT, Size(6,6));
                    morphologyEx(mask,mask, MORPH_ERODE, element3);
 //                   imshow("mask",mask);
                    rectangle(imageSource,Point(90,75),Point(210,225),Scalar(255,0,0),3);
                    waitKey(1);

                    yaw_ctrl=-9;
                    x_ctrl=0;
                    y_ctrl=0;
                    z_ctrl=0;
                    int count_gray=0;
                    int rows_gray=mask.rows;
                    int cols_gray=mask.cols*mask.channels();
                    for(int i=(rows_gray*1/4); i<(rows_gray*3/4); i++)
                    {
                        uchar *p=mask.ptr<uchar>(i);
                        for(int j=(cols_gray*3/10); j<(cols_gray*7/10);j++)
                        {
                            if(p[j]!=0)
                            {
                                count_gray++;
                            }
                        }
                    }
cout<<"count_gray: "<<count_gray<<endl;
                    if(count_gray>=170)
                    {
                        count_found++;
                        if(count_found>=2)
                        {
                            yaw_ctrl=0;
                            count_found=0;
                            fsm=-4;
                        }
                    }
                    else
                        count_found=0;
                    count_gray=0;

                    msg_velocity.pose.position.x=x_ctrl;
	                msg_velocity.pose.position.y=y_ctrl;
	                msg_velocity.pose.position.z=z_ctrl;
                    msg_velocity.pose.orientation.w=yaw_ctrl;
	                tomain_publisher.publish(msg_velocity);
                    //publish
                }


                else if(fsm==(-4))
                {
                yaw_ctrl=0;
                    x_ctrl=0;
                    y_ctrl=0;
                    z_ctrl=0;
                    count_fsm4++;
                    if(count_fsm4>=20)
                    {
                    count_fsm4=0;
                    fsm=-1;
                    }
                    msg_velocity.pose.position.x=x_ctrl;
	                msg_velocity.pose.position.y=y_ctrl;
	                msg_velocity.pose.position.z=z_ctrl;
                    msg_velocity.pose.orientation.w=yaw_ctrl;
	                tomain_publisher.publish(msg_velocity);
                }

                else if(fsm==(-1))
                {
                    Mat binary=Mat::ones(show_d.rows,show_d.cols,CV_8UC1)*255;
                    select_depth(gray,narr(120),narr(260));
                    cvtColor(show_r,show_r, COLOR_RGB2GRAY);
                    show_r.copyTo(binary,gray);
//imshow("binary-10", binary);
                    threshold(binary,binary,60,255,THRESH_BINARY);
//imshow("binary-11", binary);
                    Mat element2=getStructuringElement(MORPH_RECT,Size(4,1));
                    morphologyEx(binary,binary,MORPH_DILATE,element2);
                   // Mat element3=getStructuringElement(MORPH_RECT,Size(3,3));
                   // morphologyEx(binary,binary,MORPH_DILATE,element3);
                    bitwise_not(binary,binary);
//             imshow("binary-12", binary);
                    float bar_dep=get_bar_dep(show_d,binary);
                    if(bz_max==1)
                        max_dep_last=max_dep_0;
                    max_dep_0=constrain(bar_dep+50,125,304);
                    if(bz_max==1)
		    {
                    if(jdz_f(max_dep_0-max_dep_last)>8)
			max_dep_0=max_dep_last;
		    }
                    bz_max=1;
cout<<" dep: "<<bar_dep<<" th_dep "<<max_dep_0<<endl;
                    if(bz_bar==0)
                    {
                        bar_dep_first=bar_dep;
                        bz_bar=1;
                    }
                    x_ctrl=0.2;
                    y_ctrl=0;
                    z_ctrl=0;
                    yaw_ctrl=0;
                    if(bar_dep<=155)
                    {
                        count_ctrl++;
                        if(count_ctrl>=5)
                        {
                            x_ctrl=0;
                            count_ctrl=0;
                            fsm=0;
                        }
                    }
                    else
                        count_ctrl=0;

                    msg_velocity.pose.position.x=x_ctrl;
	                msg_velocity.pose.position.y=y_ctrl;
	                msg_velocity.pose.position.z=z_ctrl;
                    msg_velocity.pose.orientation.w=yaw_ctrl;
	                tomain_publisher.publish(msg_velocity);
                    //publish
                }

                else if(fsm==0)
                {
                    z_ctrl=0.2;
                    printf("posz_t265_local:%.3f\n",posz_t265_local);
                    int bbb=1;
                    if(posz_t265_local>1.075)
//if(bbb!=0)  //debug
                    {

                        count_ctrl++;
                        if(count_ctrl>=2)  //debug
                        {
                            z_ctrl=0;
                            count_ctrl=0;
                            fsm=1;
                        }
                    }
                    else
                        count_ctrl=0;
                    
                    msg_velocity.pose.position.x=x_ctrl;
	                msg_velocity.pose.position.y=y_ctrl;
	                msg_velocity.pose.position.z=z_ctrl;
                    msg_velocity.pose.orientation.w=yaw_ctrl;
	                tomain_publisher.publish(msg_velocity);
                    //publish
                }

                else if(fsm==1)
                {

                    int bz_legth_ok=0,bz_leafave_ok=0,bz_areaave_ok=0,bz_center_ok=0,bz_area_ok=0,bz_iscenter_ok=0,bz_leaf_ok=0;

                //depth
                    Mat gray_2=gray.clone();
                    Mat binary=Mat::ones(show_d.rows,show_d.cols,CV_8UC1)*255;
                    select_depth(gray_2,narr(100),narr(200));
                    cvtColor(show_r,show_r, COLOR_RGB2GRAY);
                    show_r.copyTo(binary,gray_2);
                        imshow("binary0", binary);
                    threshold(binary,binary,60,255,THRESH_BINARY);
                        Mat element0=getStructuringElement(MORPH_RECT,Size(5,1));
                    morphologyEx(binary,binary,MORPH_DILATE,element0);
                    bitwise_not(binary,binary);
                    imshow("binary",binary);
                    float bar_dep=get_bar_dep(show_d,binary);
                    max_dep_1=constrain(bar_dep+50,110,304);
                    piancha_x=bar_dep-155;
                    piancha_yaw=get_err_line(binary);
                    yaw_ctrl=constrain_float(piancha_yaw*0.1,11);
                    x_ctrl=constrain_float(piancha_x*0.013,0.17);
                    z_ctrl=0;
                    y_ctrl=0.07;

                    if(bz_bar2cer==0)
                    {
                        msg_velocity.pose.position.x=x_ctrl;
                        msg_velocity.pose.position.y=y_ctrl;
                        msg_velocity.pose.position.z=z_ctrl;
                        msg_velocity.pose.orientation.w=yaw_ctrl;
                        tomain_publisher.publish(msg_velocity);
                        printf("111ctrlx:%.3f ctrly:%.3f ctrlz:%.3f ctrlyaw: %.3f\n",x_ctrl,y_ctrl,z_ctrl,yaw_ctrl);
                    }

                //color
                    Mat binary_2=Mat::ones(show_d.rows,show_d.cols,CV_8UC1)*255;
                    select_depth(gray,narr(120),narr(190));
                    Mat element=getStructuringElement(MORPH_RECT,Size(6,6));
                    morphologyEx(gray,gray,MORPH_DILATE,element);
                    GaussianBlur(cir,cir,Size(5,5),3,3);
                    cir.copyTo(binary_2,gray);
                    Mat hsv;
                    int h_l=15,s_l=10,v_l=80,h_h=82,s_h=97,v_h=255;
                    cvtColor(binary_2, hsv, COLOR_BGR2HSV);//转为HSV
                    Mat1b mask;
                    inRange(hsv,Scalar(h_l,s_l,v_l),Scalar(h_h,s_h,v_h),mask);
                    Mat element1 = getStructuringElement(MORPH_RECT, Size(5,5));
                    morphologyEx(mask,mask, MORPH_ERODE, element1);
                    Mat element2 = getStructuringElement(MORPH_RECT, Size(4,4));
                    morphologyEx(mask,mask, MORPH_DILATE, element2);
                    Mat element3 = getStructuringElement(MORPH_RECT, Size(6,6));
                    morphologyEx(mask,mask, MORPH_ERODE, element3);
                    Mat element4 = getStructuringElement(MORPH_ELLIPSE, Size(5,5));
                    morphologyEx(mask,mask, MORPH_ERODE, element4);

                    imshow("mask",mask);


                    Canny(mask,image,100,230,5,false);
                    vector<vector<Point>> contours;
                    vector<Vec4i> hierarchy;
                    findContours(image,contours,hierarchy,CV_RETR_EXTERNAL,CHAIN_APPROX_SIMPLE,Point());
                    vector<Rect> boundRect(contours.size());  //定义外接矩形集合
                    vector<RotatedRect> box(contours.size()); //定义最小外接矩形集合
                    Point2f rect[4];
                    //vector<Point2f> center_all; vector<float> radius_all;
                    //vector<Point2f>
                    //vector<int> area;
                    vector<MY_cts> cts_all;
                    int count_cts=0;
                    for(int i=0; i<contours.size(); i++)
                    {
                        MY_cts cts;
                       // cout<<"size:::"<<contours[i].size()<<endl;
                        float angle;
                        int area_i=contourArea(contours[i]);
                        if(area_i>180)//mianji
                        {
                            cts.index=count_cts;
                            count_cts++;
                            cts.area=area_i;
			                cts.ct=contours[i];
                            //area.push_back((int)(contours[i].size()));
                            box[i] = minAreaRect(Mat(contours[i]));  //计算每个轮廓最小外接矩形
                            boundRect[i] = boundingRect(Mat(contours[i]));
                            circle(imageSource, Point(box[i].center.x, box[i].center.y), 5, Scalar(0, 255, 0), -1, 8);  //绘制最小外接矩形的中心点
                            cts.center.x=box[i].center.x;
                            cts.center.y=image.rows-box[i].center.y;
                            box[i].points(rect);  //把最小外接矩形四个端点复制给rect数组
                            for(int i=0;i<4;i++)
                            {
                            rect[i].y=image.rows-rect[i].y;
                            }
                            float legth_1=getlinelegth(rect[0].x,rect[0].y,rect[1].x,rect[1].y);
                            float legth_2=getlinelegth(rect[1].x,rect[1].y,rect[2].x,rect[2].y);
                            if(legth_1>=legth_2)
                            {
                                angle=getangle(rect[0].x,rect[0].y,rect[1].x,rect[1].y);
                            }
                            else
                            {
                                angle=getangle(rect[1].x,rect[1].y,rect[2].x,rect[2].y);
                            }
                            cts.angle=angle;

                            Point2f center;
                            float radius;
                            minEnclosingCircle(contours[i],center,radius);
                            //center_all.push_back(center);
                            cts.radius=radius;
                            circle(imageSource,center,radius,Scalar(255,0,0),2);
                            cts_all.push_back(cts);
                        }
                    }
                    if(cts_all.size()!=0)
                    {
                        ////sort
                        for(int i=0;i<(cts_all.size()-1);i++)
                        {
                            for(int j=i+1;j<cts_all.size();j++)
                            {
                                if((cts_all[i].area-cts_all[j].area)<0)
                                {
                                    MY_cts temp_cts=cts_all[i];
                                    cts_all[i]=cts_all[j];
                                    cts_all[j]=temp_cts;
                                }
                            }
                        }
                        ////select and delete
                        for(int i=0;i<cts_all.size();i++)
                        {
                            for(int j=i+1;j<cts_all.size();j++)
                            {
                                if(((jdz_i)(cts_all[i].center.x-cts_all[j].center.x)<=5)&&((jdz_i)(cts_all[i].center.y-cts_all[j].center.y)<=5))
                                {
                                    cts_all.erase(cts_all.begin()+j);
                                }
                            }
                        }
                        ////find min_len
   //                     int min_len=1000;
                        if(cts_all.size()>=3)
                        {
                            float jiao[3];
                            float dep_leaf[3];
                            for(int i=0;i<3;i++)
                            {
                                size_t count_ct = cts_all[i].ct.size();
                                if( count_ct < 6 )
                                    continue;

                                RotatedRect box_4 = fitEllipse(cts_all[i].ct);
                                if( MAX(box_4.size.width, box_4.size.height) > MIN(box_4.size.width, box_4.size.height)*30 )
                                    continue;
                                Mat ell=Mat::zeros(Size(300,300), CV_8UC1);
                                ellipse(ell, box_4, Scalar(255), -1, 8);
                                Mat ell_pure;
                                mask.copyTo(ell_pure,ell);
                                dep_leaf[i]=get_bar_dep(show_d,ell_pure);
                            }
                            printf("leaf1:%.2f leaf2:%.2f leaf3:%.2f   bar:%.2f\n",dep_leaf[0],dep_leaf[1],dep_leaf[2],bar_dep);
                            float dep_leaf_ave=(dep_leaf[0]+dep_leaf[1]+dep_leaf[2])/3;

                            Mat debug_1=Mat::ones(Size(300,300), CV_8UC1)*255;
                            rectangle(debug_1,Point(30,300-dep_leaf[0]),Point(80,299),Scalar(0),-1,LINE_8,0);
                            rectangle(debug_1,Point(130,300-dep_leaf[1]),Point(180,299),Scalar(0),-1,LINE_8,0);
                            rectangle(debug_1,Point(230,300-dep_leaf[2]),Point(280,299),Scalar(0),-1,LINE_8,0);
                            rectangle(debug_1,Point(100,300-dep_leaf_ave),Point(110,299),Scalar(0),-1,LINE_8,0);

                            ////find the inner cir by 3 points
                            Point2i center_inner;
                            Point2i center_real;
                            center_real.x=0;center_real.y=0;
                            float radius_inner;
                            //getcir(cts_all[0].center,cts_all[1].center,cts_all[2].center,center_inner,&radius_inner);
                            int iscir=getcir(cts_all[0].center,cts_all[1].center,cts_all[2].center,center_inner,&radius_inner);
                            if(iscir==0)
                                printf("no cir");
                            else
                            {
                                circle(imageSource, Point(center_inner.x,imageSource.rows-center_inner.y), radius_inner, Scalar(160,160,0), 2);
                                for(int i=3;i<cts_all.size();i++)
                                {
                                    float legth_center=getlinelegth((int)(center_inner.x),(int)(center_inner.y),cts_all[i].center.x,cts_all[i].center.y);
                                    if(legth_center<(radius_inner*0.55))
                                    {
                                        center_real=cts_all[i].center;
                                        break;
                                    }
                                }

                                float legth_center_1=getlinelegth((int)(center_inner.x),(int)(center_inner.y),cts_all[0].center.x,cts_all[0].center.y);
                                float legth_center_2=getlinelegth((int)(center_inner.x),(int)(center_inner.y),cts_all[1].center.x,cts_all[1].center.y);
                                float legth_center_3=getlinelegth((int)(center_inner.x),(int)(center_inner.y),cts_all[2].center.x,cts_all[2].center.y);
                                float legth_center_ave=(legth_center_1+legth_center_2+legth_center_3)/3;
                                int area_ave=(cts_all[0].area+cts_all[1].area+cts_all[2].area)/3;
                                int area_min_err=(int)(area_ave*0.8);
                                printf("area1:%d  area2:%d  area3:%d  area4:%d\n",cts_all[0].area,cts_all[1].area,cts_all[2].area,cts_all[3].area);
                             //       printf("line1:%.3f  line2:%.3f  line3:%.3f  ave:%.3f area:%d\n",legth_center_1,legth_center_2,legth_center_3,legth_center_ave,cts_all[3].area);
                             //       printf("jdz:%d  %d \n",jdz_i(cts_all[3].center.x-imageSource.cols/2),jdz_i(cts_all[3].center.y-imageSource.rows/2));
                                rectangle(debug_1,Point(200,20),Point(210,40),Scalar(0),2,LINE_8,0);
                                rectangle(debug_1,Point(200,60),Point(210,80),Scalar(0),2,LINE_8,0);
                                rectangle(debug_1,Point(200,100),Point(210,120),Scalar(0),2,LINE_8,0);
                                rectangle(debug_1,Point(200,140),Point(210,160),Scalar(0),2,LINE_8,0);
                                rectangle(debug_1,Point(200,180),Point(210,200),Scalar(0),2,LINE_8,0);
                                rectangle(debug_1,Point(200,220),Point(210,240),Scalar(0),2,LINE_8,0);
                                rectangle(debug_1,Point(200,260),Point(210,280),Scalar(0),2,LINE_8,0);
                                if((jdz_f(legth_center_1-legth_center_ave)<10)&&(jdz_f(legth_center_3-legth_center_ave)<10)&&(jdz_f(legth_center_3-legth_center_ave)<10))
                                {
                                    bz_legth_ok=1;
                                    rectangle(debug_1,Point(200,20),Point(210,40),Scalar(0),-1,LINE_8,0);
                                }
                                if((center_real.x!=0)&&(center_real.y!=0))
                                {
                                    bz_iscenter_ok=1;
                                    rectangle(debug_1,Point(200,220),Point(210,240),Scalar(0),-1,LINE_8,0);
                                }
                                if((jdz_i(cts_all[3].center.x-imageSource.cols/2)<70) && (jdz_i(cts_all[3].center.y-imageSource.rows/2)<70) &&(bz_iscenter_ok==1))
                                {
                                    bz_center_ok=1;
                                    rectangle(debug_1,Point(200,60),Point(210,80),Scalar(0),-1,LINE_8,0);
                                }
                                if(((jdz_i)(cts_all[0].area-area_ave)<area_min_err)&&((jdz_i)(cts_all[1].area-area_ave)<area_min_err)&&((jdz_i)(cts_all[2].area-area_ave)<area_min_err))
                                {
                                    bz_areaave_ok=1;
                                    rectangle(debug_1,Point(200,100),Point(210,120),Scalar(0),-1,LINE_8,0);
                                }
                                if(cts_all[3].area<400)
                                {
                                    bz_area_ok=1;
                                    rectangle(debug_1,Point(200,140),Point(210,160),Scalar(0),-1,LINE_8,0);
                                }
                                if((jdz_f(dep_leaf[0]-dep_leaf_ave)<9)&&(jdz_f(dep_leaf[1]-dep_leaf_ave)<9)&&(jdz_f(dep_leaf[2]-dep_leaf_ave)<9))
                                {
                                    bz_leafave_ok=1;
                                    rectangle(debug_1,Point(200,180),Point(210,200),Scalar(0),-1,LINE_8,0);
                                }
                                if(((dep_leaf[0]>129)&&(dep_leaf[0]<155))&&((dep_leaf[1]>129)&&(dep_leaf[1]<155))&&((dep_leaf[2]>129)&&(dep_leaf[2]<155)))
                                {
                                    bz_leaf_ok=1;
                                    rectangle(debug_1,Point(200,260),Point(210,280),Scalar(0),-1,LINE_8,0);
                                }

                                    if((bz_leafave_ok==1)&&(bz_legth_ok==1)&&(bz_center_ok==1)&&(bz_areaave_ok==1)&&(bz_area_ok==1) &&(bz_leaf_ok==1))
                                    {
                                        printf("okokokokokokokoo\n");
                                        rectangle(debug_1,Point(0,0),Point(15,100),Scalar(0),-1,LINE_8,0);
                                        if(bz_start==0)
                                        {
                                            bz_start=1;
                                            timecount=0;
                                        }
                                        if(bz_start==1)
                                        {
                                            if(timecount>0.21)
                                            {
                                                bz_start=0;
                                                timecount=0;
                                                yaw_ctrl=0;
                                                x_ctrl=0.17;                         //go ahead
                                                fsm=2;
                                            }
                                        }
                                    }
                                    else
                                    {
                                        bz_start=0;
                                        timecount=0;
                                    }
  printf("ctrlx:%.3f ctrly:%.3f ctrlz:%.3f ctrlyaw: %.3f\n",x_ctrl,y_ctrl,z_ctrl,yaw_ctrl);
 //                               }
                            }
imshow("debug_1",debug_1);
                        }
                    }
                }

                else if(fsm==2)//go ahead to 60cm
                {
		    Mat binary_2=Mat::ones(show_d.rows,show_d.cols,CV_8UC1)*255;
                    select_depth(gray,narr(20),narr(200));
                    Mat element=getStructuringElement(MORPH_RECT,Size(6,6));
                    morphologyEx(gray,gray,MORPH_DILATE,element);
                    cir.copyTo(binary_2,gray);
                    Mat hsv;
                    int h_l=15,s_l=10,v_l=80,h_h=82,s_h=97,v_h=255;
                    cvtColor(binary_2, hsv, COLOR_BGR2HSV);//转为HSV
                    Mat1b mask;
                    inRange(hsv,Scalar(h_l,s_l,v_l),Scalar(h_h,s_h,v_h),mask);
                    Mat element1 = getStructuringElement(MORPH_RECT, Size(5,5));
                    morphologyEx(mask,mask, MORPH_ERODE, element1);
                    Mat element2 = getStructuringElement(MORPH_RECT, Size(4,4));
                    morphologyEx(mask,mask, MORPH_DILATE, element2);
                    Mat element3 = getStructuringElement(MORPH_RECT, Size(6,6));
                    morphologyEx(mask,mask, MORPH_ERODE, element3);
                    imshow("mask_fsm22222",mask);
                    gray=mask.clone();
                    float center_dep=get_center_dep(show_d,gray);
                    printf("center_dep:%.2f\n",center_dep);
                    max_dep=constrain(center_dep+50,60,300);
                    x_ctrl=0.17;
                    y_ctrl=0;
                    z_ctrl=0;
                    yaw_ctrl=0;

                    Moments m=moments(mask,true);
                    Point2i center_g(m.m10/m.m00,m.m01/m.m00);
                    printf("xxx:%d  yyy:%d\n",center_g.x,center_g.y);
                    circle(imageSource,center_g,6,Scalar(130,170,0),-1,8);
                    piancha_y=mask.cols/2-center_g.x;
                    piancha_z=mask.rows/2-center_g.y;
                    y_ctrl=constrain_float(piancha_y*0.004,0.2);
                    z_ctrl=constrain_float(piancha_z*0.004,0.2);

                    if(center_dep<=65)
                    {
                        count_ctrl++;
                        if(count_ctrl>=2)
                        {
                            x_ctrl=0;
                            count_ctrl=0;
                            fsm=3;
                        }
                    }
                    else
                        count_ctrl=0;
                    
                    msg_velocity.pose.position.x=x_ctrl;
	                msg_velocity.pose.position.y=y_ctrl;
	                msg_velocity.pose.position.z=z_ctrl;
                    msg_velocity.pose.orientation.w=yaw_ctrl;
	                tomain_publisher.publish(msg_velocity);
                    //publish
                }


                else if(fsm==3)  //three leaf
                {
                    int col_now=0,row_now=0;
                    int row_min=0,row_max=0,col_min=0,col_max=0;
		            Mat binary_2=Mat::ones(show_d.rows,show_d.cols,CV_8UC1)*255;
                    Mat binary_3=Mat::ones(show_d.rows,show_d.cols,CV_8UC1)*255;
                    select_depth(gray,narr(20),narr(100));
                    Mat gray_3=gray.clone();
                    Mat element=getStructuringElement(MORPH_RECT,Size(6,6));
                    morphologyEx(gray,gray,MORPH_DILATE,element);
                    show_r.copyTo(binary_2,gray);
                    Mat hsv;
                    int h_l=15,s_l=10,v_l=80,h_h=82,s_h=97,v_h=255;
                    cvtColor(binary_2, hsv, COLOR_BGR2HSV);//转为HSV
                    Mat1b mask;
                    inRange(hsv,Scalar(h_l,s_l,v_l),Scalar(h_h,s_h,v_h),mask);
                    Mat element1 = getStructuringElement(MORPH_RECT, Size(5,5));
                    morphologyEx(mask,mask, MORPH_ERODE, element1);
                    Mat element2 = getStructuringElement(MORPH_RECT, Size(4,4));
                    morphologyEx(mask,mask, MORPH_DILATE, element2);
                    Mat element3 = getStructuringElement(MORPH_RECT, Size(6,6));
                    morphologyEx(mask,mask, MORPH_ERODE, element3);
                    imshow("mask",mask);
                    cout<<"fsm_inner"<<fsm_inner<<endl;
                    gray=mask.clone();
                    //Mat element_fsm=getStructuringElement(MORPH_RECT,Size(3,3));
                    //morphologyEx(gray,gray,MORPH_DILATE,element_fsm);
                    float center_dep=get_bar_dep(show_d,gray);
                    piancha_x=center_dep-65;
                    x_ctrl=constrain_float(piancha_x*0.02,0.2);
                    yaw_ctrl=0;
                    if(bz_red==0)
                    {
                        int is_red=find_red(show_r);
                        if(is_red==1)
                        {
                            msgtopy.data = 1;
                            sendtopy.publish(msgtopy);
                            bz_red=1;
                            imwrite("/home/opencv/catkin_ws/shot/red/red_point.jpg", show_r);
                        }
                    }
                    if(bz_red==1)
                    {
                        count_ring++;
                        if(count_ring>=60)
                        {
                            count_ring=0;
                            bz_red=2;
                            msgtopy.data = 5;
                            sendtopy.publish(msgtopy);
                        }
                    }

                    if(fsm_inner==1)
                    {
                        for(int i= (gray.cols-1);i>0;i--)
                        {
                            for(int j=0; j<gray.rows;j++)
                            {
                                if(gray.at<uchar>(j,i)!=0)
                                {
                                    col_now=i;
                                    row_min=j;
                                    break;
                                }
                            }
                            for(int j=(gray.rows-1); j>0;j--)
                            {
                                if(gray.at<uchar>(j,i)!=0)
                                {
                                    row_max=j;
                                    goto end_leaf_11;
                                }
                            }
                        }
                        end_leaf_11:
                        row_now=(row_min+row_max)/2;

                        if((jdz_i(row_now-gray.rows/2)<110)&&(jdz_i(col_now-gray.cols/2)<110))
                        {
printf("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&\n");
                            if(bz_start==0)
                            {
                                bz_start=1;
                                timecount=0;
                            }
                            if(bz_start==1)
                            {
                                if(timecount>1.1)
                                {
                                    bz_start=0;
                                    timecount=0;
                                    fsm_inner=2;
                                }
                            }
                        }
                        else
                        {
                            bz_start=0;
                            timecount=0;
                            col_now=gray.cols-1;
                        }
                    }
                    else if(fsm_inner==2)
                    {
                        for(int j=0;j<gray.rows;j++)
                        {
                            for(int i=0; i<gray.cols;i++)
                            {
                                if(gray.at<uchar>(j,i)!=0)
                                {
                                    row_now=j;
                                    col_min=i;
                                    break;
                                }
                            }
                            for(int i=(gray.cols-1); i>0;i--)
                            {
                                if(gray.at<uchar>(j,i)!=0)
                                {
                                    col_max=i;
                                    goto end_leaf_12;
                                }
                            }
                        }
                        end_leaf_12:
                        col_now=(col_min+col_max)/2;

                        if((jdz_i(row_now-gray.rows/2)<115)&&(jdz_i(col_now-gray.cols/2)<115))
                        {
printf("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&\n");
                            if(bz_start==0)
                            {
                                bz_start=1;
                                timecount=0;
                            }
                            if(bz_start==1)
                            {
                                if(timecount>1.1)
                                {
                                    bz_start=0;
                                    timecount=0;
                                    fsm_inner=3;
                                }
                            }
                        }
                        else
                        {
                            bz_start=0;
                            timecount=0;
                        }
                    }
                    if(fsm_inner==3)
                    {
                        for(int i= 0;i<gray.cols;i++)
                        {
                            for(int j=0; j<gray.rows;j++)
                            {
                                if(gray.at<uchar>(j,i)!=0)
                                {
                                    col_now=i;
                                    row_min=j;
                                    break;
                                }
                            }
                            for(int j=(gray.rows-1); j>0;j--)
                            {
                                if(gray.at<uchar>(j,i)!=0)
                                {
                                    row_max=j;
                                    goto end_leaf_13;
                                }
                            }
                        }
                        end_leaf_13:
                        row_now=(row_min+row_max)/2;

                if(bz_2to3==0)
                {
                    cvtColor(show_r,show_r, COLOR_RGB2GRAY);
                    show_r.copyTo(binary_3,gray_3);
//imshow("binary-10", binary);
                    threshold(binary_3,binary_3,60,255,THRESH_BINARY);
//imshow("binary-11", binary);
                    Mat element7=getStructuringElement(MORPH_RECT,Size(4,1));
                    morphologyEx(binary_3,binary_3,MORPH_DILATE,element7);
                   // Mat element3=getStructuringElement(MORPH_RECT,Size(3,3));
                   // morphologyEx(binary,binary,MORPH_DILATE,element3);
                    bitwise_not(binary_3,binary_3);
                    imshow("binary-11", binary_3);
                }
                int ctimg=countimage(binary_3);
                cout<<"ctimg"<<ctimg<<" fps:"<<fps<<" bz_2to3"<<bz_2to3<<endl;
                if(ctimg>=60)
                    count_2to3++;
                if(count_2to3>=50)
                    bz_2to3=1;


                if((jdz_i(row_now-gray.rows/2)<100)&&(jdz_i(col_now-gray.cols/2)<100)&&(bz_2to3==1))
                {
                printf("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&\n");
                    if(bz_start==0)
                    {
                        bz_start=1;
                        timecount=0;
                    }
                    if(bz_start==1)
                    {
                        if(timecount>1.1)
                        {
                            bz_start=0;
                            timecount=0;
                            cvtomp.data=2;
                            number_publisher2.publish(cvtomp);
                            fsm=4;
                            goto end_state_1;
                        }
                    }
                }
                else
                {
                    bz_start=0;
                    timecount=0;
                    col_now=50;
                }
                if(bz_2to3==0)
                {
                    row_now=260;
                Moments m=moments(mask,true);
                Point2i center_g(m.m10/m.m00,m.m01/m.m00);
                printf("xxx:%d  yyy:%d\n",center_g.x,center_g.y);
                col_now=(int)((float)(center_g.x)*1.3);
                }
                if(row_now<140)
                row_now=140;
                                    }
                if(row_now<299)
                    circle(imageSource, Point(col_now,row_now), 4, Scalar(200,0,200), -1, 8);
                piancha_y=(gray.cols/2)-col_now;
                piancha_z=(gray.rows/2)-row_now;
                y_ctrl=constrain_float(piancha_y*0.001,0.06);
                z_ctrl=constrain_float(piancha_z*0.001,0.06);
                printf("ctrlx:%.3f ctrly:%.3f ctrlz:%.3f ctrlyaw: %.3f\n",x_ctrl,y_ctrl,z_ctrl,yaw_ctrl);
                msg_velocity.pose.position.x=x_ctrl;
                msg_velocity.pose.position.y=y_ctrl;
                msg_velocity.pose.position.z=z_ctrl;
                msg_velocity.pose.orientation.w=yaw_ctrl;
                tomain_publisher.publish(msg_velocity);
                    //pulish
                count_shot++;
                if(count_shot>=30)
                {
                    count_shot=0;
                    number_shot++;
                    char tt[5]="";
                    string str0="/home/opencv/catkin_ws/shot/all/",str1,str2=".jpg";
                    sprintf(tt,"%d",number_shot);
                    str1=tt;
                    string str=str0+str1+str2;
                    cout<<"str::"<<str<<endl;
                    imwrite(str, show_r);
                }

                }
            }

            end_state_1:
            if(mpstate==2)
            {
printf("state:222222222222222");
                if(fsm==4)
                {
                    Mat binary=Mat::ones(show_d.rows,show_d.cols,CV_8UC1)*255;
                    select_depth(gray,narr(20),narr(280));
                    cvtColor(show_r,show_r, COLOR_RGB2GRAY);
                    show_r.copyTo(binary,gray);
                        //imshow("binary", binary);
                    threshold(binary,binary,60,255,THRESH_BINARY);
                        Mat element2=getStructuringElement(MORPH_RECT,Size(5,1));
                    morphologyEx(binary,binary,MORPH_DILATE,element2);
                    bitwise_not(binary,binary);
                    float bar_dep=get_bar_dep(show_d,binary);
                    max_dep_1=constrain(bar_dep+50,90,304);
                    min_dep_1=constrain(bar_dep-50,25,50);
                    x_ctrl=-0.2;
                    y_ctrl=0;
                    z_ctrl=0;
                    yaw_ctrl=0;

                    if(bar_dep>=(bar_dep_first-5))
                    {
                        count_ctrl++;
                        if(count_ctrl>=5)
                        {
                            x_ctrl=0;
                            count_ctrl=0;
                            fsm=5;
                            cvtomp.data=1;
                            for(int i=0;i<5;i++)
                            {
                                pub_dptosd.publish(cvtomp);
                                ros::Duration(0.02).sleep();
                            }
                        }
                    }
                    else
                        count_ctrl=0;

                    msg_velocity.pose.position.x=x_ctrl;
	                msg_velocity.pose.position.y=y_ctrl;
	                msg_velocity.pose.position.z=z_ctrl;
                    msg_velocity.pose.orientation.w=yaw_ctrl;
	                tomain_publisher.publish(msg_velocity);
                    //publish
                }

                else if(fsm==5)
                {
                    Mat binary=Mat::ones(show_d.rows,show_d.cols,CV_8UC1)*255;
                    select_depth(gray,narr(140),narr(280));
                    cvtColor(show_r,show_r, COLOR_RGB2GRAY);
                    show_r.copyTo(binary,gray);
                        //imshow("binary", binary);
                    threshold(binary,binary,60,255,THRESH_BINARY);
                        Mat element2=getStructuringElement(MORPH_RECT,Size(5,1));
                    morphologyEx(binary,binary,MORPH_DILATE,element2);
                    bitwise_not(binary,binary);
                    float bar_dep=get_bar_dep(show_d,binary);
                    max_dep_1=constrain(bar_dep+50,145,320);
                    piancha_x=bar_dep-bar_dep_first;
                    piancha_yaw=get_err_line(binary);
                    yaw_ctrl=constrain_float(piancha_yaw*0.13,13);
                    x_ctrl=constrain_float(piancha_x*0.013,0.13);
                    z_ctrl=0;
                    y_ctrl=-0.07;
                    msg_velocity.pose.position.x=x_ctrl;
	                msg_velocity.pose.position.y=y_ctrl;
	                msg_velocity.pose.position.z=z_ctrl;
                    msg_velocity.pose.orientation.w=yaw_ctrl;
	                tomain_publisher.publish(msg_velocity);
                    //publish

                    if(bz_found_land==5)
                    {
                        fsm=6;
//debug
//  cvtomp.data=3;
//  number_publisher2.publish(cvtomp);
                    }
                }

                else if(fsm==6)
                {
                    x_ctrl=ctrl_land_x;
                    y_ctrl=ctrl_land_y;
                    z_ctrl=0;
                    yaw_ctrl=0;
printf("have received::::::XXXXX:%.3f,,yyyyy:%.3f  bz_found_land:%d\n",x_ctrl,y_ctrl,bz_found_land);
                    if(bz_found_land==6)
                    {
                        fsm=7;
                        cvtomp.data=3;
                        number_publisher2.publish(cvtomp);
                    }
                    msg_velocity.pose.position.x=x_ctrl;
	                msg_velocity.pose.position.y=y_ctrl;
	                msg_velocity.pose.position.z=z_ctrl;
                    msg_velocity.pose.orientation.w=yaw_ctrl;
	                tomain_publisher.publish(msg_velocity);
                }

                else if(fsm==7)
                {
                    ;
                }
            }

            cout<<"***********************"<<endl;
            imshow("dst", imageSource);
        }
        waitKey(1);
        rate.sleep();
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        fps = 1.0 / t;
        timecount+=t;
    }
    return 0;
}

