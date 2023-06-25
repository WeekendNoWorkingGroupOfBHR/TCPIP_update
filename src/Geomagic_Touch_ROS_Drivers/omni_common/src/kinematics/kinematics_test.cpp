#include "kinematics_ur.h"
#include <iostream>
int main()
{
	double q1 = 2.35;
	double q2 = -0.95;
	double q3 = -2.02;
	double q4 = -1.14;
	double q5 = 1.58;
	double q6 = -0.05;
	// 0.108 -0.2639 0.4876 0.10988 -0.38593 0.87351 -0.27559
	std::cout<<"test inverse_kinematics, enter targe valves, include x, y, z, rx, ry, rz:\n";
	std::cout<<"test kinematics, enter joints valves, include q1, q2, q3, q4, q5, q6:\n";

	/////// ur_ rotate_vector test/////////////////////////////////////////////////////////
	double a,v[6];
	for(int i=0;i<6;i++)
	{
		std::cin>>a;//输入目标位姿x,y,z,rx,ry,rz
		v[i] = a;
	}
	std::cout<<"the values are "<<v[0]<<" "<<v[1]<<" "<<v[2]<<" "<<v[3]<<" "<<v[4]<<" "<<v[5]<<std::endl;
	kinematics_ur obj;

	/////////////inverse//////////////////
	obj.inverse_kinematics(q1,q2,q3,q4,q5,q6,v[0],v[1],v[2],v[3],v[4],v[5]);
	std::cout<<"the results are "<<q1<<" "<<q2<<" "<<q3<<" "<<q4<<" "<<q5<<" "<<q6<<std::endl;
	std::cout<<"the results are "<<q1*180/3.14<<" "<<q2*180/3.14<<" "<<q3*180/3.14<<" "<<q4*180/3.14<<" "<<q5*180/3.14<<" "<<q6*180/3.14<<std::endl;
	/////////////////////////////////////

	///////////////kinematics///////////////////
	// Eigen::Matrix4d matrix;	
	// obj.kinematics(matrix,v[0],v[1],v[2],v[3],v[4],v[5]);
	// Eigen::Matrix3d rotOfmatrix;
	// rotOfmatrix = matrix.leftCols(3).topRows(3);
	// Eigen::Quaterniond quaRot(rotOfmatrix);
	// std::cout<<"the results are "<< matrix(0,3) << " "<<matrix(1,3)<< " " <<matrix(2,3)<<std::endl;
	// std::cout<<"the results are "<< quaRot.w() << " "<<quaRot.x()<< " " << quaRot.y()<< " " << quaRot.z()<<std::endl;
	// ///////////////////////////////////////////////////////////////


	///////////quaternion parameter test:
	// double a,v[7];
	// for(int i=0;i<7;i++)
	// {
	// 	std::cin>>a;
	// 	v[i] = a;
	// }

	// kinematics_ur obj2;
	// double qrw,qrx,qry,qrz;
	// qrw = v[3];
	// qrx = v[4];
	// qry = v[5];
	// qrz = v[6];
	// Eigen::Quaterniond quaternion(qrw,qrx,qry,qrz);
	// // Eigen::Matrix3d rot_qua;
	// // rot_qua = quaternion.toRotationMatrix();
	// // std::cout<<rot_qua<<std::endl;
	// Eigen::AngleAxisd rotation_vector(quaternion);
	// double alpha;
	// Eigen::Vector3d rota_vec;
	// rota_vec = rotation_vector.axis();
	// alpha = rotation_vector.angle();
	// obj2.inverse_kinematics(q1,q2,q3,q4,q5,q6,v[0],v[1],v[2],alpha * rota_vec(0),alpha * rota_vec(1),alpha * rota_vec(2));
	// std::cout<<"the results are "<<q1<<" "<<q2<<" "<<q3<<" "<<q4<<" "<<q5<<" "<<q6<<std::endl;
	return 0;
}
