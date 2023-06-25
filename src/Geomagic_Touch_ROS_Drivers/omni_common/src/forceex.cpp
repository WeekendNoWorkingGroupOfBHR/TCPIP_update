

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
geometry_msgs::Twist ur_msg;
geometry_msgs::Twist master_msg;
geometry_msgs::Twist sensor_msg;

Eigen::Matrix3d rotation_matrix;
double force_data[6];
bool iswrench = false;
bool isFirstCall = true;
int force_flag = 0;

double touch_last_x, touch_last_y, touch_last_z;
double touch_last_rx, touch_last_ry, touch_last_rz,touch_last_rw;

double origin_x, origin_y, origin_z, origin_rx, origin_ry, origin_rz;

double q_data[6];
double q_ik[6];
double q_last[6];
double qdot[6];
double qdot_last[6];
double qddot[6];
double qdot_qp[6];
double qdot_last_qp[6];
double qddot_qp[6];
double m_dCurPos[6];
double m_dCurAng[6];
double q_origin[6];
float theta;
float theta_last;
double rotation[3];
std::vector<double> next_q;
std::vector<double> now_q;
std::vector<double> last_q;
std::vector<double> desire_pp;
double error_value_temp;
ros::Publisher force_pub;
ros::Publisher ur_publisher;
ros::Publisher master_publisher;
ros::Publisher ur_force;
double q1,q2,q3,q4,q5,q6;
double ttt = 0.005;

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
void trans_v2m(double &rx,double &ry,double&rz,Eigen::Matrix3d &rotation_matrix)//输入旋转向量，返回旋转矩阵
{
    double rotation_origin[3] = {rx,ry,rz};
    Eigen::Vector3d rotation_vector_origin = Eigen::Map<Eigen::Vector3d, Eigen::Unaligned>(rotation_origin);
    double rotation_theta_origin=rotation_vector_origin.norm();//旋转向量对应的旋转角
    Eigen::AngleAxisd rotation_a_origin (rotation_theta_origin,rotation_vector_origin/rotation_theta_origin);
    rotation_matrix=rotation_a_origin.matrix();//旋转矩阵
    // Eigen::Matrix3d temp;
    // temp <<
    //     cos(m_dCurAng[0]),  -sin(m_dCurAng[0]),  0, 
    //     sin(m_dCurAng[0]),   cos(m_dCurAng[0]),  0, 
    //             0,           0,  1;
    // rotation_matrix = temp*rotation_matrix;
}


robotiq_ft_sensor::ft_sensor force_pub_msg;//发送给主手的消息
Eigen::Vector3d force_initial_vector;
Eigen::Matrix3d rotation_matrix_initial;
Eigen::Matrix4d rotation_matrix_initial_4;
double force_initial[3],force_d[3],force_now[3];

void doforce(const robotiq_ft_sensor::ft_sensor::ConstPtr& force_msg){
    double force_gain=0.05;

    if(force_flag==0){

    force_initial[0] = force_msg->Fx*force_gain;
    force_initial[1] = force_msg->Fy*force_gain;
    force_initial[2] = force_msg->Fz*force_gain;
    force_initial_vector = Eigen::Map<Eigen::Vector3d, Eigen::Unaligned>(force_initial);
    ROS_INFO("initial force:%.2f,%.2f,%.2f",force_initial[0],force_initial[1],force_initial[2]);
    // kinematics_obj.kinematics(rotation_matrix_initial_4,m_dCurAng[0],m_dCurAng[1],m_dCurAng[2],m_dCurAng[3],m_dCurAng[4],m_dCurAng[5]);
    // rotation_matrix_initial=rotation_matrix_initial_4.leftCols(3).topRows(3);
    // // trans_v2m(origin_rx,origin_ry,origin_rz,rotation_matrix_initial);
    // force_initial_vector = rotation_matrix_initial * force_initial_vector;//求出力在世界坐标系下的表达

    force_flag=1;
    }

    // ROS_INFO("initial force unchanged:%.2f,%.2f,%.2f",force_initial[0],force_initial[1],force_initial[2]);
    // ROS_INFO("initial force:%.2f,%.2f,%.2f",force_initial_vector(0),force_initial_vector(1),force_initial_vector(2));

    // ROS_INFO("matrix:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
    // rotation_matrix_now(0),rotation_matrix_now(1),rotation_matrix_now(2),rotation_matrix_now(3),rotation_matrix_now(4),rotation_matrix_now(5),rotation_matrix_now(6),rotation_matrix_now(7),rotation_matrix_now(8));
    // trans_v2m(m_dCurPos[3],m_dCurPos[4],m_dCurPos[5],rotation_matrix_now_r);
    // ROS_INFO("matrix_r:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
    // rotation_matrix_now_r(0),rotation_matrix_now_r(1),rotation_matrix_now_r(2),rotation_matrix_now_r(3),rotation_matrix_now_r(4),rotation_matrix_now_r(5),rotation_matrix_now_r(6),rotation_matrix_now_r(7),rotation_matrix_now_r(8));

    force_now[0]=force_msg->Fx * force_gain;
    force_now[1]=force_msg->Fy * force_gain;
    force_now[2]=force_msg->Fz * force_gain;


    Eigen::Vector3d force_now_vector = Eigen::Map<Eigen::Vector3d, Eigen::Unaligned>(force_now);
    // force_now_vector = rotation_matrix_now* force_now_vector;//求当前力在世界坐标系下的表达
    // ROS_INFO("force_now_unchanged:%.2f,%.2f,%.2f",force_now[0],force_now[1],force_now[2]);
    // ROS_INFO("force_now:%.2f,%.2f,%.2f",force_now_vector(0),force_now_vector(1),force_now_vector(2));
    // double force_now_theta = force_now_vector.norm();
    // double force_initial_theta = force_initial_vector.norm();
    // ROS_INFO("initial_theta:%.2f",force_initial_theta);
    // ROS_INFO("now_theta:%.2f",force_now_theta);
    force_d[0]=force_now_vector(0)-force_initial_vector(0);
    force_d[1]=force_now_vector(1)-force_initial_vector(1);
    force_d[2]=force_now_vector(2)-force_initial_vector(2);

    std::cout<<"sensor_x:"<<force_d[0]*20<<std::endl;
    std::cout<<"sensor_y:"<<force_d[1]*20<<std::endl;
    std::cout<<"sensor_z:"<<force_d[2]*20<<std::endl;

    ur_msg.linear.x=force_d[0]*20;
    ur_msg.linear.y=force_d[1]*20;
    ur_msg.linear.z=force_d[2]*20;
    ur_force.publish(ur_msg);

    //当前的力减去初始的力就得到额外的力
    Eigen::Matrix3d rotation_matrix_now;
    Eigen::Matrix3d rotation_matrix_now_r;//根据旋转向量算的旋转矩阵
    Eigen::Matrix4d rotation_matrix_now_4;
    kinematics_obj.kinematics(rotation_matrix_now_4,m_dCurAng[0],m_dCurAng[1],m_dCurAng[2],m_dCurAng[3],m_dCurAng[4],m_dCurAng[5]);
    rotation_matrix_now=rotation_matrix_now_4.leftCols(3).topRows(3);
    // ROS_INFO("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",m_dCurAng[0],m_dCurAng[1],m_dCurAng[2],m_dCurAng[3],m_dCurAng[4],m_dCurAng[5]);

    // ROS_INFO("force_d:%.2f,%.2f,%.2f",force_d[0],force_d[1],force_d[2]);
    Eigen::Vector3d force_d_vector = Eigen::Map<Eigen::Vector3d, Eigen::Unaligned>(force_d);
    force_d_vector = rotation_matrix_now * force_d_vector;

    //发送数据给主手
    for(int k=0;k<3;k++){
      if(force_d_vector(k)>1)
      {
        force_d_vector(k)=1;
      }
      if(force_d_vector(k)<-1)
      {
        force_d_vector(k)=-1;
      }
      }
    force_pub_msg.Fx = force_d_vector(0);
    force_pub_msg.Fy = force_d_vector(1);
    force_pub_msg.Fz = force_d_vector(2);
    ROS_INFO("force_pub_msg:%.2f,%.2f,%.2f",force_pub_msg.Fx,force_pub_msg.Fy,force_pub_msg.Fz);
    force_pub.publish(force_pub_msg);
    
  }

/// @brief 
/// @param msg 
void tele_callback(const geometry_msgs::Twist & msg)
{
  if(isFirstCall)
    { 
    //   ros::Duration t(5);
    // t.sleep();
      //主手的xyz rx ry rz
      touch_last_x = msg.linear.x;
      touch_last_y = msg.linear.y;
      touch_last_z = msg.linear.z;
      touch_last_rx = rotation[0];
      touch_last_ry = rotation[1];
      touch_last_rz = rotation[2];
      touch_last_rw = rotation[3];

      q_origin[0]=m_dCurAng[0];
      q_origin[1]=m_dCurAng[1];
      q_origin[2]=m_dCurAng[2];
      q_origin[3]=m_dCurAng[3];
      q_origin[4]=m_dCurAng[4];
      q_origin[5]=m_dCurAng[5];

      // touch_last_rx=msg.angular.x;
      // touch_last_ry=msg.angular.y;
      // touch_last_rz=msg.angular.z;

      //机械臂的xyz rx ry rz
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
      next_q.clear();
	    for(int i=0;i<6;i++) {next_q.push_back(m_dCurAng[i]);}

    }
else
    {     

      //q 通过正运动学结算求得机械臂当前的R和p
      Eigen::Matrix4d rotation_matrix_origin_4;           
      kinematics_obj.kinematics(rotation_matrix_origin_4,q_origin[0],q_origin[1],q_origin[2],q_origin[3],q_origin[4],q_origin[5]);

      Eigen::Matrix3d rotation_matrix_origin;
      rotation_matrix_origin = rotation_matrix_origin_4.leftCols(3).topRows(3);
      // trans_v2m(origin_rx,origin_ry,origin_rz,rotation_matrix_origin);
      // double rotation_origin[3] = {origin_rx,origin_ry,origin_rz};
      // //将 double 数组转换为 Eigin::Vector3d型
      // Eigen::Vector3d rotation_vector_origin = Eigen::Map<Eigen::Vector3d, Eigen::Unaligned>(rotation_origin);
      // double rotation_theta_origin=rotation_vector_origin.norm();//旋转向量对应的旋转角
      // // ROS_INFO("%.2f",rotation_theta_origin);
      // Eigen::AngleAxisd rotation_a_origin (rotation_theta_origin,rotation_vector_origin/rotation_theta_origin);
      // rotation_matrix_origin=rotation_a_origin.matrix();//机械臂初始的旋转矩阵
      q_data[0] = -(msg.linear.x - touch_last_x)*0.001 + rotation_matrix_origin_4(12);
      q_data[1] = -(msg.linear.y - touch_last_y)*0.001 + rotation_matrix_origin_4(13);
      q_data[2] = (msg.linear.z - touch_last_z)*0.001 + rotation_matrix_origin_4(14);

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
      // rotation_matrix_d = rotatez_45.inverse()* trans*rotation_matrix_now*rotatez_45;
      rotation_matrix_d = trans*rotation_matrix_now;

      // Eigen::Matrix3d rotation_matrix_d = rotation_matrix_last.colPivHouseholderQr().solve(rotation_matrix_now);
      // ROS_INFO("transmatrix:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",rotation_matrix_d(0),rotation_matrix_d(1),rotation_matrix_d(2),rotation_matrix_d(3),rotation_matrix_d(4),rotation_matrix_d(5),rotation_matrix_d(6),rotation_matrix_d(7),rotation_matrix_d(8));



      //机械臂目标旋转矩阵
      Eigen::Matrix3d rotation_matrix_end;
      rotation_matrix_end = rotation_matrix_origin * rotation_matrix_d;

      Eigen::Matrix3d rotation_matrix_n;//机械臂现在的旋转矩阵
      trans_v2m(m_dCurPos[3],m_dCurPos[4],m_dCurPos[5],rotation_matrix_n);

      double m_dCurp[3]={m_dCurPos[0],m_dCurPos[1],m_dCurPos[2]};//机械臂当前的xyz
      Eigen::Vector3d m_dCurp_vector= Eigen::Map<Eigen::Vector3d, Eigen::Unaligned>(m_dCurp);


      //将机械臂移动后的旋转矩阵转换为旋转向量
      Eigen::AngleAxisd rotation_vector_end;
      rotation_vector_end.fromRotationMatrix(rotation_matrix_end);
      float theta_end  = rotation_vector_end.angle();
      Eigen::Vector3d axis = rotation_vector_end.axis();
      q_data[3]=0;
      q_data[4]=3.14;
      q_data[5]=0;

  
      //逆运动学解算的各关节角的角速度和角加速度
      for(int i=0;i<6;i++){
        // q_last[i]=q_ik[i];
        // qdot_last[i]=qdot[i];

        qdot_last_qp[i]=qdot_qp[i];
      }
      // kinematics_obj.inverse_kinematics(q_ik[1],q_ik[2],q_ik[3],q_ik[4],q_ik[5],q_ik[6],q_data[0],q_data[1],q_data[2],q_data[3],q_data[4],q_data[5]);
      
      q1=m_dCurAng[0];
      q2=m_dCurAng[1];
      q3=m_dCurAng[2];
      q4=m_dCurAng[3];
      q5=m_dCurAng[4];
      q6=m_dCurAng[5];

      //q为弧度制
      // ROS_INFO("q:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",q1,q2,q3,q4,q5,q6);

      desire_pp.clear();
	    for(int i=0;i<6;i++) {desire_pp.push_back(q_data[i]);}
      // now_q.clear();
      // for(int i=0;i<6;i++) {now_q.push_back(m_dCurAng[i]);}

	    Eigen::Matrix4d targetMatrix;
    	Eigen::Matrix4d nowMatrix;
	    Eigen::VectorXd error(6);
    	double error_value;

    	kinematics_obj.CaltargetMatrix(targetMatrix,desire_pp[0],desire_pp[1],desire_pp[2],
										desire_pp[3],desire_pp[4],desire_pp[5]);

      // ROS_INFO("endmatrix:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",rotation_matrix_end(0),rotation_matrix_end(1),rotation_matrix_end(2),rotation_matrix_end(3),rotation_matrix_end(4),rotation_matrix_end(5),rotation_matrix_end(6),rotation_matrix_end(7),rotation_matrix_end(8));
      // ROS_INFO("target_matrix:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",targetMatrix(0),targetMatrix(1),targetMatrix(2),targetMatrix(4),targetMatrix(5),targetMatrix(6),targetMatrix(8),targetMatrix(9),targetMatrix(10));
      // ROS_INFO("targetmatrix:%.2f,%.2f,%.2f",targetMatrix(12),targetMatrix(13),targetMatrix(14));
      // ROS_INFO("q_data:%.2f,%.2f,%.2f",q_data[0],q_data[1],q_data[2]);
      kinematics_obj.kinematics(nowMatrix,q1,q2,q3,q4,q5,q6);

      // ROS_INFO("now_xyz:%.2f,%.2f,%.2f",nowMatrix(12),nowMatrix(13),nowMatrix(14));
    	kinematics_obj.Error(error,targetMatrix,nowMatrix);
	    error_value = error.norm();
	    std::cout<<error_value<<std::endl;

      

	    task_obj.QP_NextQdd(error_value,next_q,now_q,last_q,desire_pp);
      // ROS_INFO("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",next_q[0],next_q[1],next_q[2],next_q[3],next_q[4],next_q[5]);
      error_value_temp=error_value;
    	last_q = now_q;
	    now_q = next_q;

      // ROS_INFO("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",q_data[0],q_data[1],q_data[2],q_data[3],q_data[4],q_data[5]);
      Eigen::Matrix4d rotation_matrix_ur;           
      kinematics_obj.kinematics(rotation_matrix_ur,m_dCurAng[0],m_dCurAng[1],m_dCurAng[2],m_dCurAng[3],m_dCurAng[4],m_dCurAng[5]);
      Eigen::Matrix3d rotation_matrix_ur3;
      rotation_matrix_ur3 = rotation_matrix_ur.leftCols(3).topRows(3);
      Eigen::AngleAxisd rotation_ur_v(rotation_matrix_ur3);
      Eigen::Vector3d rotation_ur_vector=rotation_ur_v.axis();
      double ur_angle=rotation_ur_v.angle();
      ur_msg.linear.x=rotation_matrix_ur(12);
      ur_msg.linear.y=rotation_matrix_ur(13);
      ur_msg.linear.z=rotation_matrix_ur(14);
      ur_msg.angular.x = ur_angle*rotation_ur_vector(0);
      ur_msg.angular.y = ur_angle*rotation_ur_vector(1);
      ur_msg.angular.z = ur_angle*rotation_ur_vector(2);
      ur_publisher.publish(ur_msg);

      Eigen::Quaterniond quaternion_pub(rotation[3],rotation[0],rotation[1],rotation[2]);//四元数
      Eigen::AngleAxisd rotation_vector_pub(quaternion_pub);
      Eigen::Vector3d rotation_v_pub = rotation_vector_pub.axis();
      double angle_pub = rotation_vector_pub.angle();
      master_msg.linear.x=msg.linear.x;
      master_msg.linear.y=msg.linear.y;
      master_msg.linear.z=msg.linear.z;
      master_msg.angular.x=angle_pub*rotation_v_pub(0);
      master_msg.angular.y=angle_pub*rotation_v_pub(1);
      master_msg.angular.z=angle_pub*rotation_v_pub(2);
      master_publisher.publish(master_msg);



    }
}

void dobutton(const omni_msgs::OmniButtonEvent::ConstPtr& button_state){
  if(button_state->white_button)//true 为 不传参
  {
    button_flag=0;//button_flag=0不传参
    isFirstCall = true;
  }
  else if(!button_state->white_button){
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
  // sprintf(cmd, "movej([""%lf,%lf,%lf,%lf,%lf,%lf""], 1.4, 0.5, 5, 0)\r\n", -1.6, -1.73, -2.2, -0.81, 1.60, -0.03);
  // sprintf(cmd, "movej([""%lf,%lf,%lf,%lf,%lf,%lf""], 1.4, 0.5, 5, 0)\r\n", 3.14, -0.95, -2.02, -1.14, 1.58, -0.05);
  sprintf(cmd, "movej([""%lf,%lf,%lf,%lf,%lf,%lf""], 1.4, 0.5, 5, 0)\r\n", 2.72, -1.62, -2.12, -0.97, 1.57, -1.98);
  std::string res = cmd;
  send(socket_cli, res.c_str(), strlen(res.c_str()),0);
  std::printf("Waiting UR5 Init...\n");
  sleep(5);

//预设的机械臂初始位姿
  q1 =  2.35;
  q2 = -0.95;
  q3 = -2.02;
  q4 = -1.14;
  q5 = 1.58;
  q6 = -0.05;
//机械臂系统设定的初始位姿
  // q1=-1.6;
  // q2=-1.73;
  // q3=-2.2;
  // q4=-0.81;
  // q5=1.60;
  // q6=-0.03;

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
  
  ros::Subscriber force_sub;
  force_sub = nh_p.subscribe<robotiq_ft_sensor::ft_sensor>("robotiq_ft_sensor",1,doforce);
  force_pub = nh_p.advertise<robotiq_ft_sensor::ft_sensor>("robotiq_ft_pub_sensor",100);

  ur_publisher = nh_p.advertise<geometry_msgs::Twist>("ur_position",100);
  master_publisher = nh_p.advertise<geometry_msgs::Twist>("master_position",100);
  ur_force = nh_p.advertise<geometry_msgs::Twist>("ur_force",100);

  double dt = 0.008;
  ros::Rate rt(1 / dt);

  auto runController = [&]() {

    char cmd[1024];
    //sprintf(cmd, "servoj([""%lf,%lf,%lf,%lf,%lf,%lf""], 0, 0, 0.1, 0.03, 300)\r\n", q[0], q[1], q[2], q[3], q[4], q[5]);
    sprintf(cmd, "servoj([""%lf,%lf,%lf,%lf,%lf,%lf""], 0, 0, 0.1, 0.03, 300)\r\n", next_q[0],next_q[1],next_q[2],next_q[3],next_q[4],next_q[5]);
    std::string res = cmd;
    send(socket_cli, res.c_str(), strlen(res.c_str()),0);



  };


  while(ros::ok())
  {


    if(button_flag)
    {
    runController();
    }
    else if(button_flag==false){
      ROS_INFO("STOP");
    }
    ros::spinOnce();
    rt.sleep();
    
  }

  return 0;
}