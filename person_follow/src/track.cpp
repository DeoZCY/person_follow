#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <unistd.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int8.h>

#include <opentracker/eco/eco.hpp>
#include <opentracker/eco/parameters.hpp>
#include <opentracker/eco/metrics.hpp>
#include <opentracker/eco/debug.hpp>

#include <person_follow/person.h>

using namespace std;
using namespace cv;
using namespace eco;

int idetectflag;        //收到人的检测信息标志位
int iecoflag;           //eco初始化结束标志为
int iobjflag;           //接收检测物体的数量
int itempflag;
int iectflag;

double ddistance_sum,ddistance_min;                           //区域内深度信息和
int itx,ity,itwidth,itheight,ieffective_pixcel; //中心点坐标，图像大小，区域内非零像素点数量
float fdistance,fdistance_ori;                  //区域内平均距离

cv_bridge::CvImagePtr cvPtrdepth;
Mat DepthImage;

cv_bridge::CvImagePtr cvPtrcolor;
Mat frame;

Mat frameDraw;

ECO ecotracker;

Rect2f ecobbox(0,0,0,0);
Rect2f detbbox(0,0,0,0);

ros::Publisher track_pub;

int msg_count = 0;

int detect_count=0;


void imageCB(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cvPtrdepth = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
  
    DepthImage = cvPtrdepth->image;
  
}

void colorCB(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cvPtrcolor = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    frame = cvPtrcolor->image;
    
}

void objectCB(const std_msgs::Int8 &msg)
{
    iobjflag = msg.data;
    if (iobjflag == 0)
    {
        detbbox.x = 0.0;
        detbbox.y = 0.0;
        detbbox.width = 0.0;
        detbbox.height = 0.0;
    }
}

void detectCB(const darknet_ros_msgs::BoundingBoxes & msg)
{
    cout<<"-------------------------"<<endl;
    cout<<"get detect info"<<endl;
    
    detbbox.x = msg.bounding_boxes.at(msg_count).xmin;
    detbbox.y = msg.bounding_boxes.at(msg_count).ymin;
    detbbox.width = msg.bounding_boxes.at(msg_count).xmax-msg.bounding_boxes.at(msg_count).xmin;
    detbbox.height = msg.bounding_boxes.at(msg_count).ymax-msg.bounding_boxes.at(msg_count).ymin;

    if((idetectflag==0)&&(iecoflag==0))
    {
        idetectflag=1;

        ecobbox.x = detbbox.x;
        ecobbox.y = detbbox.y;
        ecobbox.width = detbbox.width;
        ecobbox.height = detbbox.height;

        cout<<"------eco init start--------"<<endl;
    }
}

float RectJoinArea(const Rect2f& box1,const Rect2f& box2,const int& flag)
{
    float minx,miny,maxx,maxy;
    
    float JoinArea,Join;
    
    minx = std::max(box1.x,box2.x);
    miny = std::max(box1.y,box2.y);
    
    maxx = std::min(box1.x+box1.width,box2.x+box2.width);
    maxy = std::min(box1.y+box1.height,box2.y+box2.height);
    
    if(flag == 0)
    {
        Join = 0.0;
    }
    else
    {
        if(maxx > minx && maxy > miny)
        {
            JoinArea = (maxx-minx)*(maxy-miny);
            Join = JoinArea/(box1.width*box2.height + box2.width*box2.height - JoinArea);
        }
        else
        {
            Join = 0.0;
        } 
    }
    
    return Join;
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"track");
    ROS_INFO("track sysytem online");
    
    ros::NodeHandle tracknh;
    
    track_pub = tracknh.advertise<person_follow::person>("/person_follow/track",1);
    
    ros::Subscriber track_imagesub = tracknh.subscribe("/camera/aligned_depth_to_color/image_raw",1,imageCB);
    ros::Subscriber track_colorsub = tracknh.subscribe("/camera/color/image_raw",1,colorCB);
    ros::Subscriber track_objectsub = tracknh.subscribe("/darknet_ros/found_object",1,objectCB);
    ros::Subscriber track_detcetsub = tracknh.subscribe("/darknet_ros/bounding_boxes",1,detectCB);
    
    while(ros::ok)
    {
                
        if(idetectflag==1 && iecoflag!=1)
        {
            eco::EcoParameters parameters;
            ROS_INFO_STREAM("parameters");

            parameters.useCnFeature = true;
            parameters.cn_features.fparams.tablename = "/usr/local/include/opentracker/eco/look_tables/CNnorm.txt";
            
            if(frame.data)
            {
                ecotracker.init(frame, ecobbox, parameters);
                ROS_INFO_STREAM("eco init");
                
                iecoflag=1;
            }    
        }
            
        if(iecoflag==1)
        {
            while (frame.data)
            {
                frame.copyTo(frameDraw);

                bool okeco = ecotracker.update(frame,ecobbox);

                if(okeco)
                {
                    rectangle(frameDraw, ecobbox, Scalar(255, 0, 255), 2, 1);
                    float Join;
                    Join = RectJoinArea(ecobbox,detbbox,iobjflag);
                    
                    cout<<"Join:"<<Join<<endl;
                    
                    if(Join <= 0.2)
                    {
                        if(itempflag < 5)
                        {
                           itempflag = itempflag+1;
                            cout<<"lost target,searching :"<<itempflag<<endl; 
                        }
                        else
                        {
                            itempflag = 0;
                            iecoflag = 0;
                            idetectflag = 0;
                            
                            cout<<"lost target,restarting system"<<endl;
                            
                            person_follow::person person_msg;
                            
                            person_msg.targetx = frame.cols/2;
                            person_msg.targety = frame.rows/2;
                            person_msg.distance = 0.0;
                            person_msg.width = frame.cols;
                            person_msg.height = frame.rows;

                            track_pub.publish(person_msg);
                            
                            break;
                        }
                    }
                    else if(Join > 0.0)
                    {
                        cout<<"found target"<<endl;
                        
                        itempflag = 0;
                        
                        itx=ecobbox.x+ecobbox.width/2;
                        ity=ecobbox.y+ecobbox.height/2;
                        itwidth = ecobbox.width/10;
                        itheight = ecobbox.height/10;

                        if(!DepthImage.empty())
                        {
                            ieffective_pixcel = 0;
                            ddistance_sum = 0;
                            iectflag = 0;

                            for(int y=ity-itheight;y<ity+itheight;y++)
                            {
                                for(int x=itx-itwidth;x<itx+itwidth;x++)
                                {
                                    if(DepthImage.at<ushort>(y,x))
                                    {
                                         
                                        //ddistance_sum+=DepthImage.at<ushort>(y,x);
                                        //ieffective_pixcel++;
                                        
                                        if(iectflag==0)
                                        {
                                            ddistance_min = DepthImage.at<ushort>(y,x);
                                            iectflag = 1; 
                                        }
                                        else
                                        {
                                            if(DepthImage.at<ushort>(y,x) < ddistance_min)
                                            {
                                                ddistance_min = DepthImage.at<ushort>(y,x);
                                            }
                                        }
                                        
                                    }
                                }
                            }

                            //fdistance = ddistance_sum/ieffective_pixcel;
                            fdistance = ddistance_min;
                            
                            if(abs(fdistance-fdistance_ori) < 4000.0)
                            {
                                fdistance_ori = fdistance;    
                            }
                            else
                            {
                                fdistance = fdistance_ori;
                            }
                             
                            cout<<"the depth is"<<fdistance<<"x:"<<itx<<"y:"<<ity<<endl;

                            person_follow::person person_msg;
                            
                            person_msg.targetx = itx;
                            person_msg.targety = ity;
                            person_msg.distance = fdistance;
                            person_msg.width = frame.cols;
                            person_msg.height = frame.rows;

                            track_pub.publish(person_msg);

                        }
                    }   
                }
                else
                {
                    putText(frameDraw, "ECO tracking failure detected", cv::Point(10, 140), FONT_HERSHEY_SIMPLEX,0.75, Scalar(255, 0, 255), 2);
                    
                    person_follow::person person_msg;
                            
                    person_msg.targetx = itx;
                    person_msg.targety = ity;
                    person_msg.distance = fdistance;
                    person_msg.width = frame.cols;
                    person_msg.height = frame.rows;

                    track_pub.publish(person_msg);
                    
                }
                if (frameDraw.data)
                {
                    imshow("eco", frameDraw);
                }
                int c = cvWaitKey(1);
                if (c != -1)
                    c = c % 256;
                if (c == 27)
                {
                    cvDestroyWindow("eco");
                    return 0;
                }
                waitKey(1);

                ros::spinOnce();

                if (!frame.data)
                {
                    break;
                }
            }
        } 
        ros::spinOnce();   
    }
    return 0;
}
