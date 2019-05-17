#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <sstream>
#include <time.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <fstream>
#include <boost/thread.hpp>  

using namespace std; 

float track_linear_pre = 0.0;
float track_angular_pre = 0.0;

int num_recv;

int res_00 = -1; //
int res_80 = -1;

int res_MO_1 = -1;
int res_MO_2 = -1;
int res_MO_3 = -1;

int res_CM_1 = -1;
int res_CM_2 = -1;
int res_CM_3 = -1;

long int recv_vl;
long int recv_vr;

double vl_r;
double vr_r;

double vline_r;
double vangular_r;

unsigned char Motor_CAN_Init1[8] = {0x01,0x00,0x00,0x40,0x00,0x00,0x00,0x00};
unsigned char Motor_CAN_Init2[8] = {0x01,0x00,0x00,0x40,0x00,0x00,0x00,0x00};

unsigned char Motor_Model[8] = {0x55,0x4d,0x00,0x00,0x02,0x00,0x00,0x00};

unsigned char Motor_EN[8] = {0X4d,0x4f,0x00,0x00,0x01,0x00,0x00,0x00};
unsigned char Motor_DisEN[8] = {0x4d,0x4f,0x00,0x00,0x00,0x00,0x00,0x00};

unsigned char Motor_BG[8] = {0x42,0x47,0x00,0x40,0x00,0x00,0x00,0x00};
unsigned char Motor_ST[8] = {0x53,0x54,0x00,0x40,0x00,0x00,0x00,0x00}; 

unsigned char Motor_FeedBack[8] = {0x56,0x58,0x00,0x40,0x00,0x00,0x00,0x00};

unsigned char recv_l[5] = {0};
unsigned char recv_r[5] = {0};
//CAN配置参数

ros::Publisher pub_track;

class Socket_can
{
    public:
        int skt;
        struct ifreq ifr;
        struct sockaddr_can addr; 
           
        int loopback;
        
        void init_can();//CAN初始化
        void Motor_Init();//电机初始化
        void mecharm_or_weapon_init();//载荷初始化
        
};

Socket_can sock;

void Socket_can::init_can()
{
    sock.skt = socket(PF_CAN,SOCK_RAW,CAN_RAW);
    strcpy(ifr.ifr_name, "can0");
    ioctl(sock.skt, SIOCGIFINDEX,&ifr);
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    bind( sock.skt, (struct sockaddr*)&addr, sizeof(addr) );
    loopback = 0; // 0 is closed , 1 is started (default)
    setsockopt(sock.skt, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));
    
    cout<<"CAN初始化完成"<<endl;
}

void Socket_can::Motor_Init()//平台电机初始化
{
    struct can_frame send_frame[5]={{0}};
    send_frame[0].can_id = 0x301;
    send_frame[0].can_dlc = 8;

    send_frame[1].can_id = 0x302;
    send_frame[1].can_dlc = 8;

    send_frame[2].can_id = 0x303;
    send_frame[2].can_dlc = 8;
    
    send_frame[3].can_id = 0x00;
    send_frame[3].can_dlc = 8;
    
    send_frame[4].can_id = 0x80;
    send_frame[4].can_dlc = 8;

    while(res_00<0)//??why the first can communication does not success? 
    {
        memcpy(send_frame[3].data,Motor_CAN_Init1,sizeof(Motor_CAN_Init1));
        res_00 = write(sock.skt, &send_frame[3], sizeof(send_frame[3])); 
        usleep(5000);
    }
    while(res_80<0)//
    {
        memcpy(send_frame[4].data,Motor_CAN_Init2,sizeof(Motor_CAN_Init2));
        res_80 = write(sock.skt, &send_frame[4], sizeof(send_frame[4]));
        usleep(5000);
    }

    while(res_CM_1<0||res_CM_2<0||res_CM_3<0)
    {
        
        memcpy(send_frame[0].data,Motor_Model,sizeof(Motor_Model)); 
        memcpy(send_frame[1].data,Motor_Model,sizeof(Motor_Model)); 
        memcpy(send_frame[2].data,Motor_Model,sizeof(Motor_Model)); 
        res_CM_1 = write(sock.skt, &send_frame[0], sizeof(send_frame[0]));
        usleep(5000);
        res_CM_2 = write(sock.skt, &send_frame[1], sizeof(send_frame[1]));
        usleep(5000);
        res_CM_3 = write(sock.skt, &send_frame[2], sizeof(send_frame[2]));
        usleep(5000);     
    }
    
    while(res_MO_1<0||res_MO_2<0||res_MO_3<0)
    {
        memcpy(send_frame[0].data,Motor_EN,sizeof(Motor_EN));  
        memcpy(send_frame[1].data,Motor_EN,sizeof(Motor_EN)); 
        memcpy(send_frame[2].data,Motor_EN,sizeof(Motor_EN)); 
        
        res_MO_1 = write(sock.skt, &send_frame[0], sizeof(send_frame[0]));
        usleep(5000);
        res_MO_2 = write(sock.skt, &send_frame[1], sizeof(send_frame[1]));
        usleep(5000); 
        res_MO_3 = write(sock.skt, &send_frame[2], sizeof(send_frame[2]));
        usleep(5000);
    }
    cout<<"平台电机初始化完成"<<endl; 

}

void ctrlCB(const geometry_msgs::TwistConstPtr & msg)
{
	struct can_frame send_frame[2]={{0}};
    send_frame[0].can_id = 0x301;//左电机
    send_frame[0].can_dlc = 8;

    send_frame[1].can_id = 0x302;//右电机
    send_frame[1].can_dlc = 8;

    
    float linear_vel,angular_vel;
    
    linear_vel = msg->linear.x;
    angular_vel = msg->angular.z;
    
    //cout<<"sudo"<<linear_vel<<"jiaosudu"<<angular_vel<<endl;
    
    if(track_linear_pre != linear_vel || track_angular_pre != angular_vel )
    {
        
        track_linear_pre = linear_vel;
        track_angular_pre = angular_vel;
        
        double vl = linear_vel - angular_vel; //the left speed
        double vr = linear_vel + angular_vel; //the right speed

        long int vel_rpm_l = 110333 * vl;    //the number of revolving circle in the left wheel 
        long int vel_rpm_r = 110333 * vr;    //the number of revolving circle in the right wheel 
        
        cout<<"left"<<vel_rpm_l<<"right"<<vel_rpm_r<<endl;

        /*the left track :ID301*/
        send_frame[0].data[0] = 0x4a;
        send_frame[0].data[1] = 0x56;
        send_frame[0].data[2] = 0x00;
        send_frame[0].data[3] = 0x00;
        send_frame[0].data[4] = vel_rpm_l&0x000000ff;
        send_frame[0].data[5] = (vel_rpm_l&0x0000ff00)>>8;
        send_frame[0].data[6] = (vel_rpm_l&0x00ff0000)>>16;
        send_frame[0].data[7] = (vel_rpm_l&0xff000000)>>24;
        usleep(100);
        write(sock.skt, &send_frame[0], sizeof(struct can_frame)); 
        usleep(5000);
        memcpy(send_frame[0].data,Motor_BG,sizeof(Motor_BG)); 
        write(sock.skt, &send_frame[0], sizeof(struct can_frame));
        usleep(5000);
        /*the right track :ID302*/
        send_frame[1].data[0] = 0x4a; 
        send_frame[1].data[1] = 0x56;
        send_frame[1].data[2] = 0x00;
        send_frame[1].data[3] = 0x00;
        send_frame[1].data[4] = vel_rpm_r&0x000000ff;
        send_frame[1].data[5] = (vel_rpm_r&0x0000ff00)>>8;
        send_frame[1].data[6] = (vel_rpm_r&0x00ff0000)>>16;
        send_frame[1].data[7] = (vel_rpm_r&0xff000000)>>24;
        usleep(100);
        write(sock.skt, &send_frame[1], sizeof(struct can_frame)); 
        usleep(5000);
        memcpy(send_frame[1].data,Motor_BG,sizeof(Motor_BG)); 
        write(sock.skt, &send_frame[1], sizeof(struct can_frame));
        usleep(5000);   
    }
    /*
    
    struct can_frame read_frame;
    
    num_recv = read( sock.skt, &read_frame, sizeof(read_frame));
    
    if(num_recv > 0)
    {
        switch(read_frame.can_id)
        {
            case 0x281:
            {
                if(read_frame.data[0] == 0x4a && read_frame.data[1] == 0x56)//vel
                {
                    recv_l[0] = 0x01;
                    recv_l[1] = read_frame.data[4];
                    recv_l[2] = read_frame.data[5];
                    recv_l[3] = read_frame.data[6];
                    recv_l[4] = read_frame.data[7];                         
                }
                break;
            }
            case 0x282:
            {
                if(read_frame.data[0] == 0x4a && read_frame.data[1] == 0x56)//vel
                {
                    recv_r[0] = 0x02;
                    recv_r[1] = read_frame.data[4];
                    recv_r[2] = read_frame.data[5];
                    recv_r[3] = read_frame.data[6];
                    recv_r[4] = read_frame.data[7];                         
                }
                break;
            }
            default:
            {
                break;
            }
        }
    }
    usleep(2000);
    
    recv_vl = (recv_l[4]<<24)+(recv_l[3]<<16)+(recv_l[2]<<8)+recv_l[1];
    recv_vr = (recv_r[4]<<24)+(recv_r[3]<<16)+(recv_r[2]<<8)+recv_r[1];
    
    vl_r = recv_vl/110333.0;
    vr_r = recv_vr/110333.0;
    
    cout<<"-------------------------"<<endl;
    cout<<"leftv"<<vl_r<<"rightv"<<vr_r<<endl;
    
    vline_r = (vl_r+vr_r)/2;
    vangular_r = (vr_r-vl_r)/2;
    
    geometry_msgs::Twist recan_twist_msg;
    
    recan_twist_msg.linear.x = vline_r;
    recan_twist_msg.angular.z = vangular_r;
    
    pub_track.publish(recan_twist_msg);
    
    */
}

int main(int argc,char ** argv) 
{
    ros::init (argc,argv,"can_socket");//节点初始化
    
	sock.init_can();//CAN初始化
	sock.Motor_Init();//the platform control;
    
    ros::NodeHandle nh;
	ros::Subscriber can_ctrlsub = nh.subscribe("/person_follow/control",1,ctrlCB);//平台行驶
    
    pub_track = nh.advertise<geometry_msgs::Twist>("/person_follow/can_socket",1);
    
    ros::spin();
}
