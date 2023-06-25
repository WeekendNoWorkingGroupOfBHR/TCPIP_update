#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>
#include <iostream>

class Rkinematics_ur
{
    public:
        double B_z_S1 = 0.095;
        double S2_y_S1 = 0.135;
        double S2_y_E = 0.11;
        double S2_z_E = 0.425;
        double E_y_W1 = 0.095;
        double E_z_W1 = 0.4;
        double W1_z_W2 = 0.09;
        double W2_y_W3 = 0.075;
        double W3_y_T = 0.00;
        // double q1, q2, q3, q4, q5, q6;
        
        
    // private:
        Rkinematics_ur();
        void kinematics(Eigen::Matrix4d &finalMatrix, double q1, 
                                double q2,double q3, double q4, double q5, double q6);
        void geometry_jacobian(Eigen::Matrix<double,6,6> &J,  double q1, 
                                double q2,double q3, double q4, double q5, double q6);
        void rotationVector(Eigen::Vector3d &rotationvector, Eigen::Matrix3d rotationMatrix);
        void inverse_kinematics(double &q1, double &q2,double &q3, double &q4, double &q5, 
                                double &q6, double x, double y, double z, double rx, double ry, double rz);

        void Error(Eigen::VectorXd &error, Eigen::Matrix4d targetMatrix, Eigen::Matrix4d nowMatrix);
        void CaltargetMatrix(Eigen::Matrix4d &targetMatrix, double x, double y, double z,
                                    double rx, double ry, double rz);
};