#include "QP_ArmEndeffectorTask.h"
#include <iostream>
int main()
{

	QP_ArmEndeffectorTask task_obj;
	kinematics_ur kinematics_obj;
	double q1 = 2.35;
	double q2 = -0.95;
	double q3 = -2.02;
	double q4 = -1.14;
	double q5 = 1.58;
	double q6 = -0.05;
		// double q1 = 0.5;
		// double q2 = -2.00;
		// double q3 = -0.42;
		// double q4 = -0.45;
		// double q5 = 1.78;
		// double q6 = -0.25;
			// double q1 = 0;
			// double q2 = 0;
			// double q3 = 0;
			// double q4 = 0;
			// double q5 = 0;
			// double q6 = 0;
	double joints[6];
	joints[0]=q1; joints[1]=q2; joints[2]=q3; joints[3]=q4; joints[4]=q5; joints[5]=q6;
	// 0.108 -0.2639 0.4876 0.10988 -0.38593 0.87351 -0.27559
	//test targets x y z rx ry rz             || the right results
	// 0.37 -0.44 0.767 3.035 -2.156 1.792    || -46 -113 -30 -64 148 -29
	// 0.263 -0.469 0.82 2.178 -2.95 2.281    || -56 -117 -21 -40 144 15
	// 0.496 -0.363 0.723 2.975 -2.191 1.816  || -31 -114 -38 -47 133 -17
	// 0.584 0.314 0.723 4.184 -0.565 1.352   || 34  -122 -26 -40 115 -18
	std::cout<<"test qp control, enter target pp: x, y, z, rx, ry, rz:\n";

	double a,v[6];
	for(int i=0;i<6;i++)
	{
		std::cin>>a;
		v[i] = a;
	}

	std::vector<double> next_q;
	for(int i=0;i<6;i++) {next_q.push_back(0);}
	std::vector<double> now_q;
	for(int i=0;i<6;i++) {now_q.push_back(joints[i]);}
	std::vector<double> last_q;
	for(int i=0;i<6;i++) {last_q.push_back(joints[i]);}
	std::vector<double> desire_pp;
	for(int i=0;i<6;i++) {desire_pp.push_back(v[i]);}

	Eigen::Matrix4d targetMatrix;
	Eigen::Matrix4d nowMatrix;
	Eigen::VectorXd error(6);
	double error_value;
	
	kinematics_obj.CaltargetMatrix(targetMatrix,desire_pp[0],desire_pp[1],desire_pp[2],
										desire_pp[3],desire_pp[4],desire_pp[5]);
	kinematics_obj.kinematics(nowMatrix,q1,q2,q3,q4,q5,q6);
	kinematics_obj.Error(error,targetMatrix,nowMatrix);
	error_value = error.norm();
	while(error_value>0.00001)
	{

		task_obj.QP_NextQdd(error_value,next_q,now_q,last_q,desire_pp);
		last_q = now_q;
		now_q = next_q;
		std::cout<<error_value<<std::endl;
	}
	Eigen::Matrix4d finalMatrix;
	kinematics_obj.kinematics(finalMatrix,next_q[0],next_q[1],next_q[2],next_q[3],next_q[4],next_q[5]);
	Eigen::Vector3d finalRotationVector;
	kinematics_obj.rotationVector(finalRotationVector,finalMatrix.topRows(3).leftCols(3));
	std::cout<<"the final P is "<<finalMatrix(0,3)<<" "<<finalMatrix(1,3)<<" "<<finalMatrix(2,3)
					<<" "<<finalRotationVector(0)<<" "<<finalRotationVector(1)<<" "<<finalRotationVector(2)<<std::endl;
	std::cout<<"the final joints are "<<next_q[0]<<" "<<next_q[1]<<" "<<next_q[2]<<" "
								<<next_q[3]<<" "<<next_q[4]<<" "<<next_q[5]<<std::endl;
	std::cout<<"in angles are "<<next_q[0]*180/3.14<<" "<<next_q[1]*180/3.14<<" "<<next_q[2]*180/3.14<<" "
								<<next_q[3]*180/3.14<<" "<<next_q[4]*180/3.14<<" "<<next_q[5]*180/3.14<<std::endl;
	std::cout<<"the inputs are "<<v[0]<<" "<<v[1]<<" "<<v[2]<<" "<<v[3]<<" "<<v[4]<<" "<<v[5]<<std::endl;

	return 0;
}
