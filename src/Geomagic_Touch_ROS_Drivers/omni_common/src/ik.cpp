#include <ros/ros.h>

#include <iostream>

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <sys/shm.h>
#include <math.h>

#include <thread>
#include <mutex>

#include <geometry_msgs/Twist.h>
#include "QP_ArmEndeffectorTask.h"

kinematics_ur kinematics_obj;
ros::Publisher qv_pub;
ros::Publisher qa_pub;
geometry_msgs::Twist ik_msg;
geometry_msgs::Twist qv_msg;
geometry_msgs::Twist qa_msg;
double p[6];
double q_ik[6];
double q_ik_last[6]={0};
double q_dot[6];
double q_dot_last[6]={0};
double q_ddot[6];
double ttt = 0.005;
bool isFirstcallq=true;
bool isFirstcallv=true;
bool isFirstcall=true;

void doik(const geometry_msgs::Twist & ik_msg){
    p[0]=ik_msg.linear.x;
    p[1]=ik_msg.linear.y;
    p[2]=ik_msg.linear.z;
    p[3]=ik_msg.angular.x;
    p[4]=ik_msg.angular.y;
    p[5]=ik_msg.angular.z;

    if(isFirstcall){
      q_ik[0] =  2.35;
      q_ik[1] = -0.95;
      q_ik[2] = -2.02;
      q_ik[3] = -1.14;
      q_ik[4] = 1.58;
      q_ik[5] = -0.05;
      isFirstcall=false;
    }
    kinematics_obj.inverse_kinematics(q_ik[0],q_ik[1],q_ik[2],q_ik[3],q_ik[4],q_ik[5],p[0],p[1],p[2],p[3],p[4],p[5]);

    std::cout<<"q1:"<<q_ik[0]<<std::endl;
    std::cout<<"q2:"<<q_ik[1]<<std::endl;
    std::cout<<"q3:"<<q_ik[2]<<std::endl;
    std::cout<<"q4:"<<q_ik[3]<<std::endl;
    std::cout<<"q5:"<<q_ik[4]<<std::endl;
    std::cout<<"q6:"<<q_ik[5]<<std::endl;

    for(int i=0;i<6;i++){

    if(isFirstcallq)
    {
      for(int i=0;i<6;i++){
      q_ik_last[i]=q_ik[i];
      }
      isFirstcallq=false;
    }


      q_dot[i]=(q_ik[i]-q_ik_last[i])/ttt;



      if(isFirstcallv)
    {
      for(int i=0;i<6;i++){
      q_dot_last[i]=q_dot[i];
      }
      isFirstcallv=false;
    }


      q_ddot[i]=(q_dot[i]-q_dot_last[i])/ttt;


    }


    qv_msg.linear.x=q_dot[0];
    qv_msg.linear.y=q_dot[1];
    qv_msg.linear.z=q_dot[2];
    qv_msg.angular.x=q_dot[3];
    qv_msg.angular.y=q_dot[4];
    qv_msg.angular.z=q_dot[5];
    qv_pub.publish(qv_msg);

    qa_msg.linear.x=q_ddot[0];
    qa_msg.linear.y=q_ddot[1];
    qa_msg.linear.z=q_ddot[2];
    qa_msg.angular.x=q_ddot[3];
    qa_msg.angular.y=q_ddot[4];
    qa_msg.angular.z=q_ddot[5];
    qa_pub.publish(qa_msg);

   
    for(int i=0;i<6;i++){
      q_ik_last[i]=q_ik[i];
      q_dot_last[i]=q_dot[i];
    }

}

int main(int argc, char *argv[])
{
    
  ros::init(argc, argv, "ik");    
  ros::NodeHandle nh_p;


  ros::Subscriber qik_sub;
  qik_sub = nh_p.subscribe("qik_v",1,doik);

  qv_pub = nh_p.advertise<geometry_msgs::Twist>("q_v",100);

  qa_pub = nh_p.advertise<geometry_msgs::Twist>("q_a",100);

while(ros::ok()){
  ros::spin();
}

    return 0;
}
