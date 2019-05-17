#include <ros/ros.h>
#include <iostream>

#include <geometry_msgs/Twist.h>
#include <person_follow/person.h>

using namespace std;

float frspeed,frangular,fdistance,fwidth,fheight,flin,fang,fcwidthl,fcwidthr;
int targetx,targety;

ros::Publisher ctrl_pub;

void trackCB(const person_follow::person & msg)
{
    cout<<"get track info"<<endl;
    
    targetx = msg.targetx;
    targety = msg.targety;
    fdistance = msg.distance;
    fwidth = msg.width;
    fheight = msg.height;
    
    fcwidthl = (fwidth/2)-(fwidth/32);
    fcwidthr = (fwidth/2)+(fwidth/32);
    
    if (fdistance < 1500)
    {
        flin = 0.0;
    }
    else if (fdistance >= 1500 && fdistance < 3500)
    {
        flin = 0.0002*(fdistance-1500);
    }
    else if (fdistance >= 3500)
    {
        flin = 0.4;
    }
    else
    {
        flin = 0.0;
    }
    
    if (targetx < fcwidthl)
    {
        fang = 0.0005*(fcwidthl-targetx);
    }
    else if (targetx > fcwidthr)
    {
        fang = -0.0005*(targetx-fcwidthr);
    }
    else
    {
        fang = 0.0;
    }
    
    geometry_msgs::Twist ctrl_twist_msg;
    
    ctrl_twist_msg.linear.x = flin;
    ctrl_twist_msg.angular.z = fang;
    
    ctrl_pub.publish(ctrl_twist_msg);
    
    cout<<"send speed info"<< flin <<"send angular info"<< fang <<endl;
}

void canCB(const geometry_msgs::TwistConstPtr & msg)
{
    frspeed = msg->linear.x;
    frangular = msg->angular.z;
    
    cout<<"get speed info"<< frspeed <<"get angular info"<< frangular <<endl;
    cout<<"-------------------"<<endl;
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"control");
    cout<<"control system online"<<endl;
    
    ros::NodeHandle ctrlnh;
    
    ros::Subscriber ctrl_tacksub = ctrlnh.subscribe("/person_follow/track",1,trackCB);
    ros::Subscriber ctrl_cansub = ctrlnh.subscribe("/person_follow/can_socket",1,canCB);
    
    ctrl_pub = ctrlnh.advertise<geometry_msgs::Twist>("/person_follow/control",1);
    
    ros::spin();
}
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         
