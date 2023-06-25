#include "kinematics_hr.h"


kinematics_ur::kinematics_ur()
{
        B_z_S1 = 0.095;
        S2_y_S1 = 0.135;
        S2_y_E = 0.11;
        S2_z_E = 0.425;
        E_y_W1 = 0.095;
        E_z_W1 = 0.4;
        W1_z_W2 = 0.09;
        W2_y_W3 = 0.075;
        W3_y_T = 0.00;
}

void kinematics_ur::kinematics(Eigen::Matrix4d &B_T_T, double q1, 
                                    double q2,double q3, double q4, double q5, double q6)
{
    // B_z_S1 = 0.095;
    Eigen::Matrix4d W_T_B;
    W_T_B <<
                1,   0,  0,  -0.027,
                0,   1,  0,    0.11,
                0,   0,  1,   0.505,
                0,   0,  0,       1;
    Eigen::Matrix4d B_T_S1;
    B_T_S1 <<
         cos(q1),   0,       sin(q1),    0,
               0,   1,             0,    0,
        -sin(q1),   0,       cos(q1),    0,
               0,   0,             0,    1;
    B_T_S1 = W_T_B * B_T_S1;
    // std::cout<<this->B_z_S1<<std::endl;
    Eigen::Matrix4d S1_T_S2;
    S1_T_S2 <<
        1,        0,          0,     0.004,
        0,  cos(q2),   -sin(q2),     0.109,
        0,  sin(q2),    cos(q2),    -0.04,
        0,        0,          0,    1;

    Eigen::Matrix4d S2_T_E;
    S2_T_E <<
        cos(q3),   -sin(q3),      0,     0,
        sin(q3),    cos(q3),      0,     0.00056,
              0,          0,      1,    -0.07,
              0,          0,      0,     1;    

    Eigen::Matrix4d E_T_W1;
    E_T_W1 <<
        cos(q4),   0,    sin(q4),        0,
              0,   1,          0,  0.00146,
       -sin(q4),   0,    cos(q4),   -0.188,
              0,   0,          0,        1;  

    Eigen::Matrix4d W1_T_W2;
    W1_T_W2 <<
        cos(q5),   0,    sin(q5),       0,
              0,   1,          0,   0.007,
       -sin(q5),   0,    cos(q5),  -0.223,
              0,   0,          0,       1;

    Eigen::Matrix4d W2_T_W3;
    W2_T_W3 <<
        1,        0,          0,    0,
        0,  cos(q6),   -sin(q6),    0,
        0,  sin(q6),    cos(q6),    0,
        0,        0,          0,    1; 

    Eigen::Matrix4d W3_T_T;
    W3_T_T <<
             1,    0,    0,    0,
             0,    1,    0,    0,
             0,    0,    1, -0.2,
             0,    0,    0,    1; 

    B_T_T = B_T_S1 * S1_T_S2 * S2_T_E * E_T_W1 * W1_T_W2 * W2_T_W3 * W3_T_T;
}

void kinematics_ur::geometry_jacobian(Eigen::Matrix<double,6,6> &J,  double q1, 
                                double q2,double q3, double q4, double q5, double q6)
{
    Eigen::Matrix4d W_T_B;
    W_T_B <<
                1,   0,  0,  -0.027,
                0,   1,  0,    0.11,
                0,   0,  1,   0.505,
                0,   0,  0,       1;
    Eigen::Matrix4d B_T_S1;
    B_T_S1 <<
         cos(q1),   0,       sin(q1),    0,
               0,   1,             0,    0,
        -sin(q1),   0,       cos(q1),    0,
               0,   0,             0,    1;
    B_T_S1 = W_T_B * B_T_S1;
    // std::cout<<this->B_z_S1<<std::endl;
    Eigen::Matrix4d S1_T_S2;
    S1_T_S2 <<
        1,        0,          0,     0.004,
        0,  cos(q2),   -sin(q2),     0.109,
        0,  sin(q2),    cos(q2),    -0.04,
        0,        0,          0,    1;

    Eigen::Matrix4d S2_T_E;
    S2_T_E <<
        cos(q3),   -sin(q3),      0,     0,
        sin(q3),    cos(q3),      0,     0.00056,
              0,          0,      1,    -0.07,
              0,          0,      0,     1;    

    Eigen::Matrix4d E_T_W1;
    E_T_W1 <<
        cos(q4),   0,    sin(q4),        0,
              0,   1,          0,  0.00146,
       -sin(q4),   0,    cos(q4),   -0.188,
              0,   0,          0,        1;  

    Eigen::Matrix4d W1_T_W2;
    W1_T_W2 <<
        cos(q5),   0,    sin(q5),       0,
              0,   1,          0,   0.007,
       -sin(q5),   0,    cos(q5),  -0.223,
              0,   0,          0,       1;

    Eigen::Matrix4d W2_T_W3;
    W2_T_W3 <<
        1,        0,          0,    0,
        0,  cos(q6),   -sin(q6),    0,
        0,  sin(q6),    cos(q6),    0,
        0,        0,          0,    1; 

    Eigen::Matrix4d W3_T_T;
    W3_T_T <<
             1,    0,    0,    0,
             0,    1,    0,    0,
             0,    0,    1, -0.2,
             0,    0,    0,    1; 


    Eigen::Vector3d a1(0,1,0);
    // Eigen::Matrix<double,3,1> a1;
    // a1 <<
    //     0,0,1;
    a1 = B_T_S1.topRows(3).leftCols(3) * a1;
    // std::cout <<a1(2)<< std::endl;

    Eigen::Vector3d a2(1,0,0);
    a2 = B_T_S1.topRows(3).leftCols(3) * S1_T_S2.topRows(3).leftCols(3) * a2;


    Eigen::Vector3d a3(0,0,1);
    a3 = B_T_S1.topRows(3).leftCols(3) * S1_T_S2.topRows(3).leftCols(3) * S2_T_E.topRows(3).leftCols(3) * a3;

    Eigen::Vector3d a4(0,1,0);
    a4 = B_T_S1.topRows(3).leftCols(3) * S1_T_S2.topRows(3).leftCols(3) * S2_T_E.topRows(3).leftCols(3) * E_T_W1.topRows(3).leftCols(3) * a4;

    Eigen::Vector3d a5(0,1,0);
    a5 = B_T_S1.topRows(3).leftCols(3) * S1_T_S2.topRows(3).leftCols(3) * S2_T_E.topRows(3).leftCols(3) * E_T_W1.topRows(3).leftCols(3) * W1_T_W2.topRows(3).leftCols(3) * a5;

    Eigen::Vector3d a6(1,0,0);
    a6 = B_T_S1.topRows(3).leftCols(3) * S1_T_S2.topRows(3).leftCols(3) * S2_T_E.topRows(3).leftCols(3) * E_T_W1.topRows(3).leftCols(3) * W1_T_W2.topRows(3).leftCols(3) * W2_T_W3.topRows(3).leftCols(3) * a6;
    

    Eigen::Vector3d p1;
    p1 = B_T_S1.topRows(3).rightCols(1);
// std::cout <<p1<< std::endl;
    Eigen::Vector3d p2;
    p2 = (B_T_S1 * S1_T_S2).topRows(3).rightCols(1);

    Eigen::Vector3d p3;
    p3 = (B_T_S1 * S1_T_S2 * S2_T_E).topRows(3).rightCols(1);

    Eigen::Vector3d p4;
    p4 = (B_T_S1 * S1_T_S2 * S2_T_E * E_T_W1).topRows(3).rightCols(1);

    Eigen::Vector3d p5;
    p5 = (B_T_S1 * S1_T_S2 * S2_T_E * E_T_W1 * W1_T_W2).topRows(3).rightCols(1);

    Eigen::Vector3d p6;
    p6 = (B_T_S1 * S1_T_S2 * S2_T_E * E_T_W1 * W1_T_W2 * W2_T_W3).topRows(3).rightCols(1);

    Eigen::Vector3d pT;
    pT = (B_T_S1 * S1_T_S2 * S2_T_E * E_T_W1 * W1_T_W2 * W2_T_W3 * W3_T_T).topRows(3).rightCols(1);

    Eigen::Vector3d temp1;
    temp1 = a1.cross(pT - p1);

    Eigen::Vector3d temp2;
    temp2 = a2.cross(pT - p2);
    Eigen::Vector3d temp3;
    temp3 = a3.cross(pT - p3);                
    Eigen::Vector3d temp4;
    temp4 = a4.cross(pT - p4);
    Eigen::Vector3d temp5;
    temp5 = a5.cross(pT - p5);
    Eigen::Vector3d temp6;
    temp6 = a6.cross(pT - p6);


    J <<
             temp1(0), temp2(0), temp3(0), temp4(0), temp5(0), temp6(0),
             temp1(1), temp2(1), temp3(1), temp4(1), temp5(1), temp6(1),
             temp1(2), temp2(2), temp3(2), temp4(2), temp5(2), temp6(2),
                a1(0),    a2(0),    a3(0),    a4(0),    a5(0),    a6(0),
                a1(1),    a2(1),    a3(1),    a4(1),    a5(1),    a6(1),
                a1(2),    a2(2),    a3(2),    a4(2),    a5(2),    a6(2);


}

void kinematics_ur::rotationVector(Eigen::Vector3d &rotationvector, Eigen::Matrix3d rot)
{
    double theta = acos((rot(0,0)+rot(1,1)+rot(2,2)-1)/2);
    // Eigen::Matrix3d E_matrix;
    // E_matrix <<
    //         1, 0, 0,
    //         0, 1, 0,
    //         0, 0, 1;

    if(theta == 0)
    {
        rotationvector <<
            0,
            0,
            0;
        
    }
    else
    {
        Eigen::Vector3d temp_vec;
        temp_vec  <<
            rot(2,1)-rot(1,2),
            rot(0,2)-rot(2,0),
            rot(1,0)-rot(0,1);

        rotationvector = (theta/(2*sin(theta))) * temp_vec;
        Eigen::Vector3d temp1;
        temp1 = rotationvector;
        //temp1.norm();
            // std::cout << theta<< std::endl;
        Eigen::AngleAxisd rotation_vector2;
        rotation_vector2.fromRotationMatrix(rot);
        double alpha;
        Eigen::Vector3d temp22;
        temp22 = rotation_vector2.axis();
        alpha = rotation_vector2.angle();
        // rotationvector = alpha * temp22;
    }
}

//origin joint :2.35, -0.95, -2.02, -1.14, 1.58, -0.05
void kinematics_ur::inverse_kinematics(double &q1, double &q2,double &q3, double &q4, double &q5, 
                                double &q6, double x, double y, double z, double rx, double ry, double rz)
{
    double lambda = 0.1;
    double q1noNan=q1,q2noNan=q2,q3noNan=q3,q4noNan=q4,q5noNan=q5,q6noNan= q6;
    double thelta = 0;
    // double judge_while=0;
	// clock_t startTime,endTime;
    
    Eigen::Matrix4d nowMatrix;
    Eigen::Matrix4d targetMatrix;
    Eigen::Matrix4d targetMatrix_step;
    Eigen::Matrix<double,6,6>  J;
    Eigen::VectorXd dq(6);
    int i = 0;
    int j =0;
    Eigen::VectorXd error(6);
    Eigen::Vector3d error_p;
    Eigen::Vector3d error_r;

    CaltargetMatrix(targetMatrix,x,y,z,rx,ry,rz);

    // startTime=clock();
    double spendTime = 0;
    while(spendTime<1000)
    {
        geometry_jacobian(J,q1,q2,q3,q4,q5,q6);

        kinematics(nowMatrix,q1,q2,q3,q4,q5,q6);
        Error(error, targetMatrix, nowMatrix);
        if(error.hasNaN())
        {
            // endTime=clock();
            // std::cout<<"Nan has occured, now return the latest numbers"<<"  costed time "<<(endTime-startTime)/1000<<"ms"<<std::endl;
            q1=q1noNan;q2=q2noNan;q3=q3noNan;q4=q4noNan;q5=q5noNan;q6=q6noNan;
            return;
        }
        q1noNan=q1,q2noNan=q2,q3noNan=q3,q4noNan=q4,q5noNan=q5,q6noNan= q6;

        Eigen::VectorXd error_temp(6);
        error_temp = error;
        if(error_temp.norm() < 0.001)
        {
            // endTime=clock();
            // std::cout<<"completed times "<<i<<" time is "<<(endTime-startTime)/1000<<" ms"<<std::endl;
            return;
        }

        Eigen::Matrix<double,6,1> error_matrix;
        error_matrix <<
            error(0), error(1), error(2), error(3), error(4), error(5);

        dq = J.inverse() * error;
        dq = 0.01 * dq;

        q1 = q1 + dq(0);
        if(q1>6.28){q1 = 6.28;}
        if(q1<-6.28){q1 = -6.28;}
        q2 = q2 + dq(1);
        if(q2>0.1){q2 = 0.1;}
        if(q2<-2.8){q2 = -2.8;}
        q3 = q3 + dq(2);
        if(q3>-0.2){q3 = -0.2;}
        if(q3<-3.0){q3 = -3.0;}
        q4 = q4 + dq(3);
        if(q4>6.28){q4 = 6.28;}
        if(q4<-6.28){q4 = -6.28;}
        q5 = q5 + dq(4);
        if(q5>6.28){q5 = 6.28;}
        if(q5<-6.28){q5 = -6.28;}
        q6 = q6 + dq(5);
        if(q6>6.28){q6 = 6.28;}
        if(q6<-6.28){q6 = -6.28;}
        
        // q1 = fmod(q1,3.1415);
        // q2 = fmod(q2,3.1415);
        // q3 = fmod(q3,3.1415);
        // q4 = fmod(q4,3.1415);
        // q5 = fmod(q5,3.1415);
        // q6 = fmod(q6,3.1415);
        // std::cout <<error(0)<< std::endl;
    
        i =i+1;
        // endTime=clock();
        // spendTime = double(endTime - startTime)/1000;
    }
    q1=q1noNan;q2=q2noNan;q3=q3noNan;q4=q4noNan;q5=q5noNan;q6=q6noNan;
    // std::cout<<"completed but outTime, can't give an inverse result. ""Calcu costed "<<double(endTime-startTime)/1000<<" ms"<<std::endl;
}

void kinematics_ur::Error(Eigen::VectorXd &error, Eigen::Matrix4d targetMatrix, Eigen::Matrix4d nowMatrix)
{
    Eigen::Matrix4d transformMatrix;
    transformMatrix = nowMatrix.inverse() * targetMatrix;
    Eigen::Vector3d p;
    p = targetMatrix.rightCols(1).topRows(3) - nowMatrix.rightCols(1).topRows(3);
    Eigen::Matrix3d r;
    r = transformMatrix.leftCols(3).topRows(3);
    Eigen::Vector3d oumiga;
    // std::cout << "pre rotationVector..."<< std::endl;
    rotationVector(oumiga,r);
    Eigen::Vector3d real_oumiga;
    real_oumiga = nowMatrix.leftCols(3).topRows(3) * oumiga;
    error.resize(6);
    error(0) = p(0);
    error(1) = p(1);
    error(2) = p(2);
    error(3) = real_oumiga(0);
    error(4) = real_oumiga(1);
    error(5) = real_oumiga(2);

                // std::cout << oumiga.norm()<<"   "<< p.norm()<< std::endl;
                
}

void kinematics_ur::CaltargetMatrix(Eigen::Matrix4d &targetMatrix, double x, double y, double z,
                                    double rx, double ry, double rz)
{
    Eigen::Matrix3d Rot;
    Eigen::Vector3d rotVector(rx,ry,rz);
    double theta = rotVector.norm();
    Eigen::Vector3d normVector;
    normVector = rotVector.normalized();
    // R=cosθI+(1−cosθ)nnT+sinθnhat
    Eigen::Matrix3d Imatrix;
    Imatrix <<
        cos(theta), 0, 0,
        0, cos(theta), 0,
        0, 0, cos(theta);
    Eigen::Matrix3d Imatrix2;
    Imatrix2 <<
        1-cos(theta), 0, 0,
        0, 1-cos(theta), 0,
        0, 0, 1-cos(theta);
    Eigen::Matrix3d Imatrix3;
    Imatrix3 <<
        sin(theta), 0, 0,
        0, sin(theta), 0,
        0, 0, sin(theta);
        
    Eigen::Matrix3d normVectorHat;
    normVectorHat<< 
         0,          -normVector(2), normVector(1),
         normVector(2),        0,    -normVector(0),
         -normVector(1), normVector(0),  0;
    Eigen::Matrix3d Imatrix4;
    Eigen::Matrix<double,3,1> normVectorMatrix;
    normVectorMatrix <<
        normVector(0), normVector(1), normVector(2);
    Eigen::Matrix<double,1,3> normVectorMatrix_transpose;
    normVectorMatrix_transpose <<
        normVector(0), normVector(1), normVector(2);
    Eigen::Matrix3d temp_;
    temp_ <<
        normVector(0)*normVector(0), normVector(0)*normVector(1), normVector(0)*normVector(2),
        normVector(1)*normVector(0), normVector(1)*normVector(1), normVector(1)*normVector(2),
        normVector(2)*normVector(0), normVector(2)*normVector(1), normVector(2)*normVector(2);

    Imatrix4 = Imatrix2 * temp_;
    Rot = Imatrix + Imatrix4 + Imatrix3 * normVectorHat;
    targetMatrix <<
            Rot(0,0),  Rot(0,1),  Rot(0,2),  x,
            Rot(1,0),  Rot(1,1),  Rot(1,2),  y,
            Rot(2,0),  Rot(2,1),  Rot(2,2),  z,
                   0,         0,         0,  1;
}