/*
 * 在teleoperation_test.cpp的基础上，尝试添加可操作度来解决奇异位形的问题
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


#define    MYPORT     30003   //端口号
#define    BUF_SIZE   10240   //数据缓冲区最大长度

char const* SERVER_IP = "192.168.137.233";
int result = 0;
int button_flag=0;
int publish_flag=0;
std::vector<double> q_temp_;
std::vector<double> force_temp;
std::mutex m;
std::mutex mc_rtc_mtx;
bool isRecv =true;
geometry_msgs::Twist force_msg;
Eigen::Matrix3d rotation_matrix;
double force_data[6];
bool iswrench = false;
bool isFirstCall = true;

double touch_last_x, touch_last_y, touch_last_z;
double touch_last_rx, touch_last_ry, touch_last_rz,touch_last_rw;

double origin_x, origin_y, origin_z, origin_rx, origin_ry, origin_rz;

double q_data[6];
double q_static[6];
double m_dCurPos[6];
double m_dCurAng[6];
float theta;
float theta_last;
double rotation[4];
std::vector<double> next_q;
std::vector<double> now_q;
std::vector<double> last_q;
std::vector<double> desire_pp;
double error_value_temp;

double q1,q2,q3,q4,q5,q6;

kinematics_ur kinematics_obj;

QP_ArmEndeffectorTask task_obj;
int pp_num=0;

double GetDouble(char* pData)
{
	double t;
	char* p = (char*)&t;
	int i;
	for (i = 0; i < 8; i++)
	{
		p[i] = pData[7 - i];
	}
	return t;
}

char* GetDword(char* pData)
{
	char* t;
	char* p = (char*)&t;
	int i;
	for (i = 0; i < 4; i++)
	{
		p[i] = pData[3 - i];
	}
	return t;
}


std::vector<double> OnRecvData(char* pData, int nLen, std::vector<double> &force_temp)
{
  std::vector<double> q_temp;
	char* dwPackLen;
	dwPackLen = GetDword((char*)pData);
	double data1[6];
	int n;
	for (n = 0; n < 6; n++)
	{
		data1[n] = GetDouble((char*)(pData + 252 + n * 8));
    q_temp.push_back(data1[n]);
	}
  	for (n = 0; n < 6; n++)
		m_dCurAng[n] = data1[n];
	//cout << "joint1: " << data1[0]*180/3.14 << " joint2: " << data1[1] * 180 / 3.14 << " joint3: " << data1[2] * 180 / 3.14 << " joint4: " << data1[3] * 180 / 3.14 << " joint5: " << data1[4] * 180 / 3.14 << " joint6: " << data1[5] * 180 / 3.14 << endl;
	
  double data2[30];
	for (n = 0; n < 30; n++)
	{
		data2[n] = GetDouble((char*)(pData + 444 + n * 8));
	}
	for (n = 0; n < 6; n++)
		m_dCurPos[n] = data2[n];
	//cout << "x: " << data2[0] << " y: " << data2[1] << " z: " << data2[2] << endl;

  double data3[6];
  std::vector<double> force_temp_;
	for (n = 0; n < 6; n++)
	{
		data3[n] = GetDouble((char*)(pData + 540 + n * 8));
    force_temp_.push_back(data3[n]);
	}
  for (n = 0; n < 6; n++)
		force_data[n] = data3[n];
  force_temp = force_temp_;
	//cout << "xforce: " << data3[0] << " yforce: " << data3[1] << " zforce: " << data3[2] << " yawM: " << data3[3] << " pitchM: " << data3[4] << " rollM: " << data3[5] << endl;
  return q_temp;

}

void t1(char* recvBuf, int nLen, int ret, int socket_cli)
{
    while (1)
    {
      if(isRecv)
      {
        std::lock_guard<std::mutex> lockGuard(m);
        memset(recvBuf, 0, nLen);
        ret = recv(socket_cli, recvBuf, nLen, 0);
        if (ret)
        {
          //std::vector<double> q_temp_;
          q_temp_ = OnRecvData(recvBuf, ret, force_temp);
          //mc_rtc::log::info("had receive info",q_temp_[0]);
        }
        // isRecv = false;
      }
    }
}

void tele_callback(const geometry_msgs::Twist & msg)
{

  
  if(isFirstCall)
    {
    
      touch_last_x = msg.linear.x;
      touch_last_y = msg.linear.y;
      touch_last_z = msg.linear.z;
      touch_last_rx = rotation[0];
      touch_last_ry = rotation[1];
      touch_last_rz = rotation[2];
      touch_last_rw = rotation[3];

      // touch_last_rx=msg.angular.x;
      // touch_last_ry=msg.angular.y;
      // touch_last_rz=msg.angular.z;

      origin_x=m_dCurPos[0];
      origin_y=m_dCurPos[1];
      origin_z=m_dCurPos[2];
      origin_rx = m_dCurPos[3];
      origin_ry = m_dCurPos[4];
      origin_rz = m_dCurPos[5];

      isFirstCall = false;

        now_q.clear();
        for(int i=0;i<6;i++) {now_q.push_back(m_dCurAng[i]);}
        last_q.clear();
	      for(int i=0;i<6;i++) {last_q.push_back(m_dCurAng[i]);}



    }
else
    {

      q_data[0] = -(msg.linear.x - touch_last_x)*0.002 + origin_x;
      q_data[1] = -(msg.linear.y - touch_last_y)*0.002 + origin_y;
      q_data[2] = (msg.linear.z - touch_last_z)*0.002 + origin_z;

      // q_data[3] = (msg.angular.x - touch_last_rx) + origin_rx;
      // q_data[4] = (msg.angular.y - touch_last_ry) + origin_ry;
      // q_data[5] = (msg.angular.z - touch_last_rz) + origin_rz;

      Eigen::Matrix3d rotation_matrix_now = rotation_matrix;//主手现在的旋转矩阵
      // ROS_INFO("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",rotation_matrix_now(0),rotation_matrix_now(1),rotation_matrix_now(2),rotation_matrix_now(3),rotation_matrix_now(4),rotation_matrix_now(5),rotation_matrix_now(6),rotation_matrix_now(7),rotation_matrix_now(8));

      Eigen::Quaterniond quaternion_last(touch_last_rw,touch_last_rx,touch_last_ry,touch_last_rz);
      Eigen::AngleAxisd rotation_vector_last(quaternion_last);
      Eigen::Matrix3d trans=rotation_vector_last.matrix().inverse();

      //主手的变换矩阵 nan
      Eigen::Matrix3d rotation_matrix_d;
      // ROS_INFO("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",rotation_matrix_d(0),rotation_matrix_d(1),rotation_matrix_d(2),rotation_matrix_d(3),rotation_matrix_d(4),rotation_matrix_d(5),rotation_matrix_d(6),rotation_matrix_d(7),rotation_matrix_d(8));
      rotation_matrix_d = trans*rotation_matrix_now;

      // Eigen::Matrix3d rotation_matrix_d = rotation_matrix_last.colPivHouseholderQr().solve(rotation_matrix_now);
      // ROS_INFO("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",rotation_matrix_d(0),rotation_matrix_d(1),rotation_matrix_d(2),rotation_matrix_d(3),rotation_matrix_d(4),rotation_matrix_d(5),rotation_matrix_d(6),rotation_matrix_d(7),rotation_matrix_d(8));
  

      
      //已知机械臂的旋转向量，求机械臂初始的旋转矩阵
      double rotation_origin[3] = {origin_rx,origin_ry,origin_rz};
      //将 double 数组转换为 Eigin::Vector3d型
      Eigen::Vector3d rotation_vector_origin = Eigen::Map<Eigen::Vector3d, Eigen::Unaligned>(rotation_origin);
      double rotation_theta_origin=rotation_vector_origin.norm();//旋转向量对应的旋转角
      // ROS_INFO("%.2f",rotation_theta_origin);
      Eigen::AngleAxisd rotation_a_origin (rotation_theta_origin,rotation_vector_origin/rotation_theta_origin);
      Eigen::Matrix3d rotation_matrix_origin;
      rotation_matrix_origin=rotation_a_origin.matrix();

      //求机械臂移动后的旋转矩阵
      Eigen::Matrix3d rotation_matrix_end;
      rotation_matrix_end = rotation_matrix_origin * rotation_matrix_d;
      // ROS_INFO("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",rotation_matrix_end(0),rotation_matrix_end(1),rotation_matrix_end(2),rotation_matrix_end(3),rotation_matrix_end(4),rotation_matrix_end(5),rotation_matrix_end(6),rotation_matrix_end(7),rotation_matrix_end(8));

      //将机械臂移动后的旋转矩阵转换为旋转向量
      Eigen::AngleAxisd rotation_vector_end;
      rotation_vector_end.fromRotationMatrix(rotation_matrix_end);
      float theta_end  = rotation_vector_end.angle();
      Eigen::Vector3d axis = rotation_vector_end.axis();
      q_data[3]=axis(0)*theta_end;
      q_data[4]=axis(1)*theta_end;
      q_data[5]=axis(2)*theta_end;

      // ROS_INFO("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",q_data[0],q_data[1],q_data[2],q_data[3],q_data[4],q_data[5]);
      // ROS_INFO("%.2f,%.2f,%.2f",q_data[3],q_data[4],q_data[5]);
      
      q1=m_dCurAng[0];
      q2=m_dCurAng[1];
      q3=m_dCurAng[2];
      q4=m_dCurAng[3];
      q5=m_dCurAng[4];
      q6=m_dCurAng[5];

      // obj.inverse_kinematics(q1,q2,q3,q4,q5,q6,q_data[0],q_data[1],q_data[2],q_data[3],q_data[4],q_data[5]);
      //q为弧度制
      // ROS_INFO("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",q1,q2,q3,q4,q5,q6);

      desire_pp.clear();
	    for(int i=0;i<6;i++) {desire_pp.push_back(q_data[i]);}

	    Eigen::Matrix4d targetMatrix;
    	Eigen::Matrix4d nowMatrix;
	    Eigen::VectorXd error(6);
    	double error_value;

    	kinematics_obj.CaltargetMatrix(targetMatrix,desire_pp[0],desire_pp[1],desire_pp[2],
										desire_pp[3],desire_pp[4],desire_pp[5]);
    	// kinematics_obj.CaltargetMatrix(targetMatrix,desire_pp[pp_num+0],desire_pp[pp_num+1],desire_pp[pp_num+2],
			// 							desire_pp[pp_num+3],desire_pp[pp_num+4],desire_pp[pp_num+5]);
      // ROS_INFO( "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",desire_pp[pp_num+0],desire_pp[pp_num+1],desire_pp[pp_num+2],desire_pp[pp_num+3],desire_pp[pp_num+4],desire_pp[pp_num+5]);
    	// kinematics_obj.kinematics(nowMatrix,now_q[0],now_q[1],now_q[2],now_q[3],now_q[4],now_q[5]);
      kinematics_obj.kinematics(nowMatrix,q1,q2,q3,q4,q5,q6);

    	kinematics_obj.Error(error,targetMatrix,nowMatrix);
	    error_value = error.norm();
	    // std::cout<<error_value<<std::endl;

	    task_obj.QP_NextQdd(error_value,next_q,now_q,last_q,desire_pp);
      // error_value_temp=error_value;//error_value_temp<0.00001
    	last_q = now_q;
	    now_q = next_q;
      ROS_INFO("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",next_q[0],next_q[1],next_q[2],next_q[3],next_q[4],next_q[5]);

      //operability
      Eigen::Matrix<double,6,6> J;//////////////J
      kinematics_obj.geometry_jacobian(J,m_dCurAng[0],m_dCurAng[1],m_dCurAng[2],m_dCurAng[3],m_dCurAng[4],m_dCurAng[5]);
      //J_w
      Eigen::Matrix<double,3,6> J_w;
      J_w=J.bottomRows(3);
      Eigen::Matrix<double,6,3> J_w_T;
      J_w_T=J_w.transpose();
      Eigen::Matrix<double,3,3> A_w;
      A_w=J_w*J_w_T;
      double miu_w;
      miu_w=A_w.determinant();
      std::cout<<sqrt(miu_w)<<std::endl;
      double detJ;
      detJ=sin(m_dCurAng[2])*sin(m_dCurAng[4])*425*392*(425*cos(m_dCurAng[1])+392*cos(m_dCurAng[1]
          +m_dCurAng[2])+94.75*sin(m_dCurAng[1]+m_dCurAng[2]+m_dCurAng[3]));
      std::cout<<detJ<<std::endl;
      // //J_v
      // Eigen::Matrix<double,3,6> J_v;
      // J_v=J.topRows(3);
      // Eigen::Matrix<double,6,3> J_v_T;
      // J_v_T=J_v.transpose();
      // Eigen::Matrix<double,3,3> A_v;
      // A_v=J_v*J_v_T;
      // double miu_v;
      // miu_v=A_v.determinant();
      // std::cout<<miu_v<<std::endl;      



      // Eigen::Matrix<double,3,4> J_1234;
      // J_1234=J.leftCols(4).bottomRows(3);
      // Eigen::Matrix<double,4,3> J_1234_t;
      // J_1234_t=J_1234.transpose();
      // Eigen::Matrix<double,3,3> A_1234;
      // A_1234=J_1234*J_1234_t;
      // double miu_1234;
      // miu_1234=A_1234.determinant();
      // std::cout<<miu_1234<<std::endl;

    }
}

void dobutton(const omni_msgs::OmniButtonEvent::ConstPtr& button_state){
  if(button_state->grey_button)//true 为 不传参
  {
    button_flag=0;//button_flag=0不传参
    isFirstCall = true;
  }
  else if(!button_state->grey_button){
    button_flag=1;//button_flag=1传参
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
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "TeleOperation_ur");    
  ros::NodeHandle nh_p;

  //ros::Publisher ur_force_pub = nh_p.advertise<geometry_msgs::Twist>("/ur/force", 1);


  std::string conf = "";

  //socket connect part///////////////////////////////////////////////////////
  char recvbuf[BUF_SIZE];
	
	/*
	 *@fuc: socket()创建套节字
	 *
	 */
	int socket_cli = socket(AF_INET, SOCK_STREAM, 0);
	if(socket_cli < 0)
	{
		std::cout << "socket() error\n";
		return -1;
	}
  	/*
	 *@fuc: 服务器端IP4地址信息,struct关键字可不写
	 *@fuc: 初始化sever地址信息   
	 */
	struct sockaddr_in sev_addr;  
	memset(&sev_addr, 0, sizeof(sev_addr));
	sev_addr.sin_family      = AF_INET;
	sev_addr.sin_port        = htons(MYPORT);
	sev_addr.sin_addr.s_addr = inet_addr(SERVER_IP);
    std::printf("connecting...\n");
	/*
	 *@fuc: 使用connect()函数来配置套节字,建立一个与TCP服务器的连接
	 */
	if(connect(socket_cli, (struct sockaddr*) &sev_addr, sizeof(sev_addr)) < 0)
	{ 
        std::printf("connect error\n");
		//return -1;
	}
	else
        std::printf("connected successfullly!\n");

  int nLen = 1220;
	char* recvBuf = new char[nLen];
	int ret;

  static std::thread th1(t1,recvBuf, nLen, ret, socket_cli);
  th1.detach();

  //socket connect part//////////////////////////////////////////////////


  char cmd[1024];
  sprintf(cmd, "movej([""%lf,%lf,%lf,%lf,%lf,%lf""], 1.4, 0.5, 5, 0)\r\n", 2.35, -0.95, -2.02, -1.14, 1.58, -0.05);
  std::string res = cmd;
  send(socket_cli, res.c_str(), strlen(res.c_str()),0);
  std::printf("Waiting UR5 Init...\n");
  sleep(5);

  q1 =  2.35;
  q2 = -0.95;
  q3 = -2.02;
  q4 = -1.14;
  q5 = 1.58;
  q6 = -0.05;

  origin_x = m_dCurPos[0];
  origin_y = m_dCurPos[1];
  origin_z = m_dCurPos[2];
  origin_rx = m_dCurPos[3];
  origin_ry = m_dCurPos[4];
  origin_rz = m_dCurPos[5];

  q_data[0] = origin_x;
  q_data[1] = origin_y;
  q_data[2] = origin_z;
  q_data[3] = origin_rx;
  q_data[4] = origin_ry;
  q_data[5] = origin_rz;


  ros::Subscriber button_state;
  button_state = nh_p.subscribe("/phantom/button",1,dobutton);

  ros::Subscriber tele_data;
  tele_data = nh_p.subscribe("/phantom/Twist_msgs", 1, tele_callback);

  ros::Subscriber rotation_sub;
  rotation_sub = nh_p.subscribe("/phantom/rotation_msg",1,dorotation);

  double dt = 0.008;
  ros::Rate rt(1 / dt);

  auto runController = [&]() {

    char cmd[1024];
    //sprintf(cmd, "servoj([""%lf,%lf,%lf,%lf,%lf,%lf""], 0, 0, 0.1, 0.03, 300)\r\n", q[0], q[1], q[2], q[3], q[4], q[5]);
    sprintf(cmd, "servoj([""%lf,%lf,%lf,%lf,%lf,%lf""], 0, 0, 0.1, 0.03, 300)\r\n", next_q[0],next_q[1],next_q[2],next_q[3],next_q[4],next_q[5]);
    std::string res = cmd;
    //降低发布频率
    // publish_flag++;
    // if(publish_flag==10){
    send(socket_cli, res.c_str(), strlen(res.c_str()),0);
    // publish_flag=0;
    // }


  };


  while(ros::ok())
  {

    if(button_flag)
    {
    runController();
    }
    else if(!button_flag){
      ROS_INFO("STOP");
    }
    
    ros::spinOnce();
    rt.sleep();
    
  }

  return 0;
}