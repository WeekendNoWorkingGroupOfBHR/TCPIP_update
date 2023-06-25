#include <iostream>
// Eigen
#include <Eigen/Dense>

// eigen-quadprog
#include "/usr/include/eigen-quadprog/QuadProg.h"

#include <vector>
#include "kinematics_ur.h"

class QP_ArmEndeffectorTask
{
    public:
        
        double t; //control time
        double k; //control stiffness
        kinematics_ur obj;
        Eigen::QuadProgDense qp;
        QP_ArmEndeffectorTask();
        void QP_NextQdd(double &error, std::vector<double> &next_q,std::vector<double> now_q, 
                            std::vector<double> last_q, std::vector<double> desire_pp);
        void ineqWithXBounds(Eigen::MatrixXd& Aineq, Eigen::VectorXd& Bineq,
                                const Eigen::VectorXd& XL, const Eigen::VectorXd& XU);
};