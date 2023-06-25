#include "QP_ArmEndeffectorTask.h"
#include <ros/ros.h>

QP_ArmEndeffectorTask::QP_ArmEndeffectorTask()
{
    //obj->kinematics_ur();
    kinematics_ur obj2;
    obj = obj2;
    Eigen::QuadProgDense qp_obj(6,6,24);
    qp = qp_obj;
    t = 0.005; //control time 8ms
    k =60; //control stiffness gain
    
}

void QP_ArmEndeffectorTask::QP_NextQdd(double &error, std::vector<double> &next_q,std::vector<double> now_q, std::vector<double> last_q, 
                                            std::vector<double> desire_pp)
{
    clock_t startTime,endTime;
    startTime=clock();
    double a; //////////////////////////// a
    a = 1 + (2*sqrt(k)*t) + 0.5*k*t*t;

    Eigen::VectorXd et(6);
    Eigen::Matrix4d now_W_T_E;

    obj.kinematics(now_W_T_E,now_q[0],now_q[1],now_q[2],now_q[3],now_q[4],now_q[5]);
        // std::cout<<now_W_T_E(1,3)<<std::endl;
    Eigen::VectorXd now_p(6);
    Eigen::Matrix4d desire_matrix;
    obj.CaltargetMatrix(desire_matrix,desire_pp[0],desire_pp[1],desire_pp[2],desire_pp[3],desire_pp[4],desire_pp[5]);
    // ROS_INFO("desirematrix:%.2f,%.2f,%.2f",desire_matrix(12),desire_matrix(13),desire_matrix(14));
    obj.Error(et,desire_matrix,now_W_T_E);
    et = -et;/////////////////////////////////et

    Eigen::Matrix<double,6,6> J;//////////////J
    obj.geometry_jacobian(J,now_q[0],now_q[1],now_q[2],now_q[3],now_q[4],now_q[5]);	
    Eigen::Matrix<double,6,6> J_last;
    obj.geometry_jacobian(J_last,last_q[0],last_q[1],last_q[2],last_q[3],last_q[4],last_q[5]);
    Eigen::Matrix<double,6,6> J_d;
    J_d = (J - J_last)/t;////////////////////////J_d
    Eigen::Matrix<double,6,6> inverseJ;
    // inverseJ = J.inverse();
    // std::cout<< J.determinant() << std::endl;

    Eigen::VectorXd qt_d(6);//v
    qt_d << 0,0,0,0,0,0;
    for(int count=0;count<6;count++)
    {qt_d(count) = (now_q[count]-last_q[count])/t;}
    // if(J.determinant()<0.0001)
    // {
    //     qt_d = qt_d * 10;
    // }
    // qt_d <<
    //     (now_q[0]-last_q[0])/t, (now_q[1]-last_q[1])/t, (now_q[2]-last_q[2])/t, 
    //     (now_q[3]-last_q[3])/t, (now_q[4]-last_q[4])/t, (now_q[5]-last_q[5])/t;
    // qt_d = qt_d/t;//////////////////////////qt_d
    Eigen::VectorXd et_d(6);
    et_d = J * qt_d;///////////////////////////et_d

    Eigen::VectorXd b;
    b = 2*sqrt(k)*et_d + k*et + k*et_d*t;/////////////////////b

    Eigen::Matrix<double,6,6> H;
    H = 2*a*a*J.transpose()*J;/////////////////////////////////H

    Eigen::VectorXd temp_calcu;
    temp_calcu = a* J_d * qt_d + b;
    Eigen::VectorXd fT = 2* temp_calcu.transpose() * a * J;/////fT

	// qp.solve(qp1.Q, qp1.C,
	// 	qp1.Aeq, qp1.Beq,
	// 	qp1.Aineq, qp1.Bineq);
    Eigen::Matrix<double,6,6> Aeq;
    Aeq <<
        0,0,0,0,0,0,
        0,0,0,0,0,0,
        0,0,0,0,0,0,
        0,0,0,0,0,0,
        0,0,0,0,0,0,
        0,0,0,0,0,0;
    Eigen::VectorXd Beq(6);
    Beq <<
        0,0,0,0,0,0;

    Eigen::MatrixXd Aineq(Eigen::MatrixXd::Zero(24,6));
    for(int k=0;k<6;k++)
    {
        Aineq(k,k) = 1;
    }
    for(int k2=0;k2<6;k2++)
    {
        Aineq(k2+6,k2) = -1;
        
    }
    for(int k3=0;k3<6;k3++)
    {
        Aineq(k3+12,k3) = 1;
        
    }
    for(int k4=0;k4<6;k4++)
    {
        Aineq(k4+18,k4) = -1;
        
    }
    Eigen::VectorXd Bineq(Eigen::VectorXd::Zero(24));
    Eigen::VectorXd XL(6);
    Eigen::VectorXd XU(6);
    Eigen::VectorXd qXL(6);
    Eigen::VectorXd qXU(6);

    XL << -3.0, -3.0, -3.0, -3.0, -3.0, -3.0;/////////////the inequal constraint of qdd
    XU << 3.0, 3.0, 3.0, 3.0, 3.0, 3.0;
    for(int l=0;l<6;l++)//0~11 constraint of qdd
    {
        Bineq(l) = XU(l);
    }
    for(int l2=0;l2<6;l2++)
    {
        Bineq(l2+6) = -XL(l2);
    }
   
    qXL << -0.78, -2.52, -3.14, -2.71, 0.1, -6.28;/////////////the inequal constraint of q
    qXU << 3.92, 0, -0.1, 0.43, 3, 6.28;
    // qXL << -6.28, -6.28, -3.14, -6.28, -6.28, -6.28;/////////////the inequal constraint of q
    // qXU << 6.28, 6.28, -0.1, 6.28, 6.28, 6.28;
    for(int l3=0;l3<6;l3++)//12~23 constraint of q
    {
        Bineq(l3+12) = 2*(qXU(l3)-now_q[l3]-qt_d(l3)*t)/(t*t);
    }
    for(int l4=0;l4<6;l4++)
    {
        Bineq(l4+18) = -2*(qXL(l4)-now_q[l4]-qt_d(l4)*t)/(t*t);
    }

    
    // ineqWithXBounds(qp1.Aineq, qp1.Bineq, XL, XU);
	qp.solve(H, fT,
		Aeq, Beq,
		Aineq, Bineq);

    // Eigen::VectorXd result;
    // double result1;
    // result = qp.result();
    // result.resize(6);
    // std::cout<<qp.result()(0)<<std::endl;
    Eigen::VectorXd qpresult(6);
    qpresult = qp.result();
    // if(J.determinant()<0.0001)
    // {
    //     qpresult = qpresult *10;
    // }
    next_q.clear();
    next_q.push_back(0.5 * qpresult(0) * t*t + qt_d(0)*t + now_q[0]);
    next_q.push_back(0.5 * qpresult(1) * t*t + qt_d(1)*t + now_q[1]);
    next_q.push_back(0.5 * qpresult(2) * t*t + qt_d(2)*t + now_q[2]);
    next_q.push_back(0.5 * qpresult(3) * t*t + qt_d(3)*t + now_q[3]);
    next_q.push_back(0.5 * qpresult(4) * t*t + qt_d(4)*t + now_q[4]);
    next_q.push_back(0.5 * qpresult(5) * t*t + qt_d(5)*t + now_q[5]);
 
    // std::cout << next_q[0] <<" "<< next_q[1] <<" "<< next_q[2] <<" "
    //     << next_q[3] <<" "<< next_q[4] <<" "<< next_q[5] <<" "<< std::endl;
    obj.kinematics(now_W_T_E,next_q[0],next_q[1],next_q[2],next_q[3],next_q[4],next_q[5]);
    obj.Error(et,desire_matrix,now_W_T_E);
    // ROS_INFO("desire:%.2f,%.2f,%.2f",desire_matrix(12),desire_matrix(13),desire_matrix(14));
    // ROS_INFO("now:%.2f,%.2f,%.2f",now_W_T_E(12),now_W_T_E(13),now_W_T_E(14));
    error = et.norm();
    // std::cout<<"error is"<<error<<std::endl;

    endTime=clock();
    // std::cout<<"error is "<<error<<", and qp spent "<<double(endTime-startTime)/1000<<"ms"<<std::endl;


}

void QP_ArmEndeffectorTask::ineqWithXBounds(Eigen::MatrixXd& Aineq, Eigen::VectorXd& Bineq,
	const Eigen::VectorXd& XL, const Eigen::VectorXd& XU)
{
	double inf = std::numeric_limits<double>::infinity();

	std::vector<std::pair<int, double> > lbounds, ubounds;

	for(int i = 0; i < XL.rows(); ++i)
	{
		if(XL[i] != -inf)
			lbounds.emplace_back(i, XL[i]);
		if(XU[i] != inf)
			ubounds.emplace_back(i, XU[i]);
	}

	long int nrconstr = Bineq.rows() + static_cast<long int>(lbounds.size()) +
		static_cast<long int>(ubounds.size());

	Eigen::MatrixXd A(Eigen::MatrixXd::Zero(nrconstr, Aineq.cols()));
	Eigen::VectorXd B(Eigen::VectorXd::Zero(nrconstr));

	A.block(0, 0, Aineq.rows(), Aineq.cols()) = Aineq;
	B.segment(0, Bineq.rows()) = Bineq;

	int start = static_cast<int>(Aineq.rows());

	for(int i = 0; i < static_cast<int>(lbounds.size()); ++i)
	{
		const auto& b = lbounds[i];
		A(start, b.first) = -1.;
		B(start) = -b.second;
		++start;
	}

	for(int i = 0; i < static_cast<int>(ubounds.size()); ++i)
	{
		const auto& b = ubounds[i];
		A(start, b.first) = 1.;
		B(start) = b.second;
		++start;
	}

	Aineq = A;
	Bineq = B;
}