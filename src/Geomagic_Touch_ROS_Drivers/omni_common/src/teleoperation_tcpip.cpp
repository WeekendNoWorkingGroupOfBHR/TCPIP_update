/*
 * tcpip通讯
 */


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

#include "omni_msgs/OmniButtonEvent.h"
#include "omni_msgs/OmniFeedback.h"
#include "omni_msgs/OmniState.h"
#include "QP_ArmEndeffectorTask.h"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include "robotiq_ft_sensor/rq_sensor_state.h"
#include "robotiq_ft_sensor/ft_sensor.h"
#include "robotiq_ft_sensor/sensor_accessor.h"

#include "TCPIP_Port.hpp"
#include <iostream>
using namespace std;

#define    MYPORT     30003   //端口号
#define    BUF_SIZE   10240   //数据缓冲区最大长度

char const* SERVER_IP = "192.168.137.233";
int result = 0;
int button_flag=0;
int button_flag_grey;
bool isRecv =true;
geometry_msgs::Twist force_msg;
Eigen::Matrix3d rotation_matrix;
Eigen::Matrix3d rotation_matrix_l;
Eigen::Matrix3d rotation_matrix_end;
double force_data[6];
bool iswrench = false;
bool isFirstCall= true;
bool isFirstCall_xyz = true;
bool isFirstCall_rxyz = true;
int force_flag = 0;

double touch_last_x, touch_last_y, touch_last_z;
double touch_last_rx, touch_last_ry, touch_last_rz,touch_last_rw;

double origin_x, origin_y, origin_z, origin_rx, origin_ry, origin_rz;
robotiq_ft_sensor::ft_sensor force_pub_msg;//发送给主手的消息
perception::Com_Data send_data;

double q_data[6];
double qdata_last[6]={0};
double m_dCurPos[6];
double m_dCurAng[6];
double q_origin[6];
float theta;
float theta_last;
double rotation[4];
std::vector<double> next_q;
std::vector<double> now_q;
std::vector<double> last_q;
std::vector<double> desire_pp;
double error_value_temp;
ros::Publisher force_pub;
double q1,q2,q3,q4,q5,q6;

kinematics_ur kinematics_obj;

QP_ArmEndeffectorTask task_obj;
int pp_num=0;

void trans_v2m(double &rx,double &ry,double&rz,Eigen::Matrix3d &rotation_matrix)//输入旋转向量，返回旋转矩阵
{
    double rotation_origin[3] = {rx,ry,rz};
    Eigen::Vector3d rotation_vector_origin = Eigen::Map<Eigen::Vector3d, Eigen::Unaligned>(rotation_origin);
    double rotation_theta_origin=rotation_vector_origin.norm();//旋转向量对应的旋转角
    Eigen::AngleAxisd rotation_a_origin (rotation_theta_origin,rotation_vector_origin/rotation_theta_origin);
    rotation_matrix=rotation_a_origin.matrix();//旋转矩阵
}

void tele_callback(const geometry_msgs::Twist & msg)
{
    if(isFirstCall_xyz)//如果灰色按下去
    {
      origin_x = msg.linear.x;
      origin_y = msg.linear.y;
      origin_z = msg.linear.z;
      qdata_last[0]=q_data[0];
      qdata_last[1]=q_data[1];
      qdata_last[2]=q_data[2];
      rotation_matrix_l=rotation_matrix_end;
      isFirstCall_xyz = false;
    }
    if(button_flag)//如果白色按下去
    {
      origin_x = msg.linear.x;
      origin_y = msg.linear.y;
      origin_z = msg.linear.z;
      qdata_last[0]=0;
      qdata_last[1]=0;
      qdata_last[2]=0;
      rotation_matrix_l<<1,0,0,0,1,0,0,0,1;
    }
    if(!(button_flag_grey||button_flag))//如果灰色和白色都没按下去
    {

    q_data[0] = (msg.linear.x - origin_x)*0.001 + qdata_last[0];
    q_data[1] = (msg.linear.y - origin_y)*0.001 + qdata_last[1];
    q_data[2] = (msg.linear.z - origin_z)*0.001 + qdata_last[2];
    
    //根据主手当前和初始的旋转矩阵求得变换的矩阵
      Eigen::Matrix3d rotation_matrix_now = rotation_matrix;//主手当前的旋转矩阵
      // ROS_INFO("now:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",rotation_matrix_now(0),rotation_matrix_now(1),rotation_matrix_now(2),rotation_matrix_now(3),rotation_matrix_now(4),rotation_matrix_now(5),rotation_matrix_now(6),rotation_matrix_now(7),rotation_matrix_now(8));

      Eigen::Quaterniond quaternion_last(touch_last_rw,touch_last_rx,touch_last_ry,touch_last_rz);
      // Eigen::AngleAxisd rotation_vector_last(quaternion_last);
      // Eigen::Matrix3d rotation_matrix_last=rotation_vector_last.matrix();
      Eigen::Matrix3d rotation_matrix_last=quaternion_last.toRotationMatrix();//主手初始的旋转矩阵
      // ROS_INFO("last:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",rotation_matrix_last(0),rotation_matrix_last(1),rotation_matrix_last(2),rotation_matrix_last(3),rotation_matrix_last(4),rotation_matrix_last(5),rotation_matrix_last(6),rotation_matrix_last(7),rotation_matrix_last(8));

      Eigen::Matrix3d trans = rotation_matrix_last.inverse();

      //主手的变换矩阵 nan
      Eigen::Matrix3d rotation_matrix_d;
      // ROS_INFO("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",rotation_matrix_d(0),rotation_matrix_d(1),rotation_matrix_d(2),rotation_matrix_d(3),rotation_matrix_d(4),rotation_matrix_d(5),rotation_matrix_d(6),rotation_matrix_d(7),rotation_matrix_d(8));
      rotation_matrix_d = trans*rotation_matrix_now;
      rotation_matrix_end = rotation_matrix_l*rotation_matrix_d;
      Eigen::AngleAxisd rotation_vector (rotation_matrix_end);
      Eigen::Vector3d rotation_v =rotation_vector.axis();
      double rotation_angle=rotation_vector.angle();
      q_data[3]=rotation_angle* rotation_v[0];
      q_data[4]=rotation_angle* rotation_v[1];
      q_data[5]=rotation_angle* rotation_v[2];
    }
}

void dobutton(const omni_msgs::OmniButtonEvent::ConstPtr& button_state){
  // if(button_state->white_button)//true 为 不传参
  // {
  //   button_flag=0;//button_flag=0不传参
  //   isFirstCall = true;
  // }
  // else if(!button_state->white_button){
  //   button_flag=1;//button_flag=1传参
  // }
  button_flag=button_state->white_button;//按下去是1，不按是0
  button_flag_grey=button_state->grey_button;
  if(button_state->grey_button)
  {
    isFirstCall_xyz=true;
    isFirstCall_rxyz=true;
  }
}

void dorotation(const geometry_msgs::Twist & rotation_msg)
{
  rotation[0]=rotation_msg.angular.x;//x
  rotation[1]=rotation_msg.angular.y;//y
  rotation[2]=rotation_msg.angular.z;//z
  rotation[3]=rotation_msg.linear.x;//w
  Eigen::Quaterniond quaternion(rotation[3],rotation[0],rotation[1],rotation[2]);//四元数
  
  //将四元数转换为旋转矩阵
  rotation_matrix=quaternion.toRotationMatrix();//主手的旋转矩阵
  
  if(isFirstCall_rxyz){
      touch_last_rx = rotation[0];
      touch_last_ry = rotation[1];
      touch_last_rz = rotation[2];
      touch_last_rw = rotation[3];
      Eigen::AngleAxisd initial_vector(quaternion);
      Eigen::Vector3d initial_v=initial_vector.axis();
      double initial_angle = initial_vector.angle();
      origin_rx = initial_angle*initial_v(0);
      origin_ry = initial_angle*initial_v(1);
      origin_rz = initial_angle*initial_v(2);
      isFirstCall_rxyz = false;
  }
  if(button_flag){
      touch_last_rx = rotation[0];
      touch_last_ry = rotation[1];
      touch_last_rz = rotation[2];
      touch_last_rw = rotation[3];
      Eigen::AngleAxisd initial_vector(quaternion);
      Eigen::Vector3d initial_v=initial_vector.axis();
      double initial_angle = initial_vector.angle();
      origin_rx = initial_angle*initial_v(0);
      origin_ry = initial_angle*initial_v(1);
      origin_rz = initial_angle*initial_v(2);
  }
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "TeleOperation_ur");    
  ros::NodeHandle nh_p;

  std::string conf = "";

  //socket connect part///////////////////////////////////////////////////////
  char recvbuf[BUF_SIZE];
	

  //socket connect part//////////////////////////////////////////////////

  perception::tcpip_port port("192.168.1.100");
  port.initial();
  perception::Com_Data send_data;
  // vector<perception::footstep> steps;
  // steps.clear();
  // if (port.receive_steps() == 0)
  //   {
  //       ROS_INFO("receive error!!!");
  //   }
  //   else
  //   {
  //       steps = port.getRecvSteps();
  //   }
  // for (auto & step : steps)
  //   {
  //       cout<<"fx_initial: "      <<step.fx      <<endl;
  //       cout<<"fy_initial: "      <<step.fy      <<endl;
  //       cout<<"fz_initial: "      <<step.fz      <<endl;
  //   }
  // double force_initial[3];
  // force_initial[0] = steps.at(0).fx;
  // force_initial[1] = steps.at(0).fy;
  // force_initial[2] = steps.at(0).fz;
  // Eigen::Vector3d force_initial_vector = Eigen::Map<Eigen::Vector3d, Eigen::Unaligned>(force_initial);

  sleep(5);

  ros::Subscriber button_state;
  button_state = nh_p.subscribe("/phantom/button",1,dobutton);

  ros::Subscriber tele_data;
  tele_data = nh_p.subscribe("/phantom/Twist_msgs", 1, tele_callback);

  ros::Subscriber rotation_sub;
  rotation_sub = nh_p.subscribe("/phantom/rotation_msg",1,dorotation);

  force_pub = nh_p.advertise<robotiq_ft_sensor::ft_sensor>("robotiq_ft_pub_sensor",100);

  double dt = 0.008;
  ros::Rate rt(1 / dt);

  rotation_matrix_l<<1,0,0,0,1,0,0,0,1;
  rotation_matrix_end<<1,0,0,0,1,0,0,0,1;

  auto runController = [&]() {
    if(button_flag==0){
    send_data.LeftoRight = 5;//left 1 , right 0
    send_data.q1 = q_data[0];
    send_data.q2 = q_data[1];
    send_data.q3 = q_data[2];
    send_data.q4 = q_data[3];
    send_data.q5 = q_data[4];
    send_data.q6 = q_data[5];}
    else if(button_flag==1){
    send_data.LeftoRight = 5;//left 1 , right 0
    send_data.q1 = 0.0;
    send_data.q2 = 0.0;
    send_data.q3 = 0.0;
    send_data.q4 = 0.0;
    send_data.q5 = 0.0;
    send_data.q6 = 0.0;
    }

    printf("\033c");
    printf("initialx:%.2f\n",origin_x);
    printf("initialy:%.2f\n",origin_y);
    printf("initialz:%.2f\n",origin_z);
    printf("initialrx:%.2f\n",origin_rx);
    printf("initialry:%.2f\n",origin_ry);
    printf("initialrz:%.2f\n",origin_rz);

    printf("----------------------------------\n");

    printf("sendx:%.2f\n",send_data.q1);
    printf("sendy:%.2f\n",send_data.q2);
    printf("sendz:%.2f\n",send_data.q3);
    printf("sendrx:%.2f\n",send_data.q4);
    printf("sendry:%.2f\n",send_data.q5);
    printf("sendrz:%.2f\n",send_data.q6);
    port.sendFlag(send_data);


  };


  while(ros::ok())
  {
    // if(button_flag)
    // {
    runController();
    // sleep(0.2);
    // }
    // else if(!button_flag){
    //   ROS_INFO("STOP");
    // }

    // steps.clear();
    // port.receive_steps();
    // steps = port.getRecvSteps();
    // for (auto & step : steps)
    // {
        // double force_now[3],force_d[3];
        // force_now[0] = step.fx;
        // force_now[1] = step.fy;
        // force_now[2] = step.fz;
        // // cout<<"fx: "      <<step.fx      <<endl;
        // // cout<<"fy: "      <<step.fy      <<endl;
        // // cout<<"fz: "      <<step.fz      <<endl;
        // Eigen::Vector3d force_now_vector = Eigen::Map<Eigen::Vector3d, Eigen::Unaligned>(force_now);
        // force_d[0]=force_now_vector(0)-force_initial_vector(0);
        // force_d[1]=force_now_vector(1)-force_initial_vector(1);
        // force_d[2]=force_now_vector(2)-force_initial_vector(2);
        // //当前的力减去初始的力就得到额外的力
        // Eigen::Matrix3d rotation_matrix_now;
        // Eigen::Matrix3d rotation_matrix_now_r;//根据旋转向量算的旋转矩阵
        // Eigen::Matrix4d rotation_matrix_now_4;
        // kinematics_obj.kinematics(rotation_matrix_now_4,m_dCurAng[0],m_dCurAng[1],m_dCurAng[2],m_dCurAng[3],m_dCurAng[4],m_dCurAng[5]);
        // rotation_matrix_now=rotation_matrix_now_4.leftCols(3).topRows(3);
        // // ROS_INFO("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",m_dCurAng[0],m_dCurAng[1],m_dCurAng[2],m_dCurAng[3],m_dCurAng[4],m_dCurAng[5]);

        // // ROS_INFO("force_d:%.2f,%.2f,%.2f",force_d[0],force_d[1],force_d[2]);
        // Eigen::Vector3d force_d_vector = Eigen::Map<Eigen::Vector3d, Eigen::Unaligned>(force_d);
        // force_d_vector = rotation_matrix_now * force_d_vector;

        // //发送数据给主手
        // force_pub_msg.Fx = force_d_vector(0);
        // force_pub_msg.Fy = force_d_vector(1);
        // force_pub_msg.Fz = force_d_vector(2);
        // // ROS_INFO("force_pub_msg:%.2f,%.2f,%.2f",force_pub_msg.Fx,force_pub_msg.Fy,force_pub_msg.Fz);
        
        // // force_pub.publish(force_pub_msg);
    // }   
    ros::spinOnce();
    rt.sleep();
    
  }

  return 0;
}