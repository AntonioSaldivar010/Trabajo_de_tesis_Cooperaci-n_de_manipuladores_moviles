#define _USE_MATH_DEFINES
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include <eigen3/Eigen/Dense>
#include <fstream>
#include<iostream>
#include <math.h>
#include <iostream>
#include <chrono>
#include "ServerComm.h"
#include <sys/time.h>   
#include <qpOASES.hpp>
#include <boost/thread.hpp>

using namespace std;
using namespace Eigen;
using boost::thread;
using boost::mutex;

USING_NAMESPACE_QPOASES;

struct timeval ti, tf;

geometry_msgs::PoseStamped Robot1_Pose;
geometry_msgs::PoseStamped Base1_Pose;
geometry_msgs::PoseStamped Objective_Pose;
geometry_msgs::PoseStamped irobot_Pose;
geometry_msgs::PoseStamped woodbase_Pose;
geometry_msgs::PoseStamped Base2_Pose;
geometry_msgs::PoseStamped Robot2_Pose;
geometry_msgs::PoseStamped Bubblebase_Pose;
geometry_msgs::PoseStamped Boxbase_Pose;

const double pi = M_PI;

VectorXd rpy_B11(3), X_11i(3), X_11d(3), rpy_B11(3), X_11i(3), X_11d(3), X_11dx(3), X_11dB(3), X_11dBx(3);

double x_d, y_d, z_d,x_dB,y_dB,z_dB,x_dBx,y_dBx,z_dBx;
double x_1i, y_1i, z_1i;
double x_1iB, y_1iB, z_1iB, Q0_1iB, Q1_1iB, Q2_1iB, Q3_1iB;

void chatterCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    //   ROS_INFO("I heard: [%s]", msg->data.c_str());
    Robot1_Pose = *msg;
}
void chatterCallback2(const geometry_msgs::PoseStamped::ConstPtr& msg2)
{
    //   ROS_INFO("I heard: [%s]", msg->data.c_str());
    Base1_Pose = *msg2;
}
void chatterCallback3(const geometry_msgs::PoseStamped::ConstPtr& msg3)
{
    //   ROS_INFO("I heard: [%s]", msg->data.c_str());
    Objective_Pose = *msg3;
}
void chatterCallback4(const geometry_msgs::PoseStamped::ConstPtr& msg4)
{
    //   ROS_INFO("I heard: [%s]", msg->data.c_str());
    irobot_Pose = *msg4;
}
void chatterCallback5(const geometry_msgs::PoseStamped::ConstPtr& msg5)
{
    //   ROS_INFO("I heard: [%s]", msg->data.c_str());
    woodbase_Pose = *msg5;
}
void chatterCallback6(const geometry_msgs::PoseStamped::ConstPtr& msg6)
{
    //   ROS_INFO("I heard: [%s]", msg->data.c_str());
    Base2_Pose = *msg6;
}
void chatterCallback7(const geometry_msgs::PoseStamped::ConstPtr& msg7)
{
    //   ROS_INFO("I heard: [%s]", msg->data.c_str());
    Robot2_Pose = *msg7;
}
void chatterCallback8(const geometry_msgs::PoseStamped::ConstPtr& msg8)
{
    //   ROS_INFO("I heard: [%s]", msg->data.c_str());
    Bubblebase_Pose = *msg8;
}
void chatterCallback9(const geometry_msgs::PoseStamped::ConstPtr& msg9)
{
    //   ROS_INFO("I heard: [%s]", msg->data.c_str());
    Boxbase_Pose = *msg9;
}

VectorXd quat2rpy(double Q0, double Q1, double Q2, double Q3)
{
    double roll, pitch, yaw;
    VectorXd RPY(3);

    roll = atan2((2 * (Q0 * Q1 + Q2 * Q3)), (1 - 2 * (pow(Q1, 2) + pow(Q2, 2))));
    pitch = asin(2 * (Q0 * Q2 - Q3 * Q1));
    yaw = atan2((2 * (Q0 * Q3 + Q1 * Q2)), (1 - 2 * (pow(Q2, 2) + pow(Q3, 2))));

    RPY << roll, pitch, yaw;

    return RPY;
}

void optitrack_readings()
{
    // Obtención de las coordenadas cartesianas y angulares del objeto de referencia
    x_d = Objective_Pose.pose.position.x;
    y_d = Objective_Pose.pose.position.y;
    z_d = Objective_Pose.pose.position.z;

    x_dB = Bubblebase_Pose.pose.position.x;
    y_dB = Bubblebase_Pose.pose.position.y;
    z_dB = Bubblebase_Pose.pose.position.z;
    
    x_dBx = Boxbase_Pose.pose.position.x;
    y_dBx = Boxbase_Pose.pose.position.y;
    z_dBx = Boxbase_Pose.pose.position.z;
    
    // Obtención de las coordenadas cartesianas y angulares del efector final del R1
    x_1i = Robot1_Pose.pose.position.x;
    y_1i = Robot1_Pose.pose.position.y;
    z_1i = Robot1_Pose.pose.position.z;

    // Obtención de las coordenadas cartesianas y angulares del efector final del R2
//     x_2i = Robot2_Pose.pose.position.x;
//     y_2i = Robot2_Pose.pose.position.y;
//     z_2i = Robot2_Pose.pose.position.z;

    // Obtención de las coordenadas cartesianas y angulares de la base del R1
    x_1iB = Base1_Pose.pose.position.x;
    y_1iB = Base1_Pose.pose.position.y;
    z_1iB = Base1_Pose.pose.position.z;
    Q0_1iB = Base1_Pose.pose.orientation.w;
    Q1_1iB = Base1_Pose.pose.orientation.x;
    Q2_1iB = Base1_Pose.pose.orientation.y;
    Q3_1iB = Base1_Pose.pose.orientation.z;

    // Obtención de las coordenadas cartesianas y angulares de la base del R2
//     x_2iB = Base2_Pose.pose.position.x;
//     y_2iB = Base2_Pose.pose.position.y;
//     z_2iB = Base2_Pose.pose.position.z;
//     Q0_2iB = Base2_Pose.pose.orientation.w;
//     Q1_2iB = Base2_Pose.pose.orientation.x;
//     Q2_2iB = Base2_Pose.pose.orientation.y;
//     Q3_2iB = Base2_Pose.pose.orientation.z;

}

void r11_coord_transformation()
{
    rpy_B11 = quat2rpy(Q0_1iB, Q1_1iB, Q2_1iB, Q3_1iB);

    X_11i << x_1i * cos(rpy_B11(2)) + y_1i * sin(rpy_B11(2)), -x_1i * sin(rpy_B11(2)) + y_1i * cos(rpy_B11(2)), z_1i;
    X_11d << x_dB * cos(rpy_B11(2)) + y_dB * sin(rpy_B11(2)), -x_dB * sin(rpy_B11(2)) + y_dB * cos(rpy_B11(2)), z_dB;
    X_11dx << x_dBx * cos(rpy_B11(2)) + y_dBx * sin(rpy_B11(2)), -x_dBx * sin(rpy_B11(2)) + y_dBx * cos(rpy_B11(2)), z_dBx;
}


Matrix3d skew(MatrixXd v)
{
    Matrix3d S;

    S << 0, -v(2), v(1),
        v(2), 0, -v(0),
        -v(1), v(0), 0;

    return S;
}

double MFsig01n(double In, double a, double b)
{
    double Mu;

    Mu = (1 / (1 + exp(-b * (-In - a))));

    return Mu;
}

double MFsig01(double In, double a, double b)
{
    double Mu;

    Mu = (1 / (1 + exp(-b * (In - a))));

    return Mu;
}

double MFgus01(double In, double a, double b)
{
    double Mu;

    Mu = exp(-(pow((In - a), 2)) / pow(b, 2));

    return Mu;
}

VectorXd Control_Kuka(Vector3d e, double t, VectorXd dq, MatrixXd JAest, VectorXd v, double ef1, VectorXd qu1, Vector3d pu1, Vector3d XDpp)
{
    MatrixXd CKK(3, 3);
    double M1PLX; double M1PSX; double M1ZEX; double M1NSX; double M1NLX;
    double M1PLY; double M1PSY; double M1ZEY; double M1NSY; double M1NLY;
    double M1PLZ; double M1PSZ; double M1ZEZ; double M1NSZ; double M1NLZ;
    double M2PLX; double M2PSX; double M2ZEX; double M2NSX; double M2NLX;
    double M2PLY; double M2PSY; double M2ZEY; double M2NSY; double M2NLY;
    double M2PLZ; double M2PSZ; double M2ZEZ; double M2NSZ; double M2NLZ;
    RowVectorXd epsxblock(1, 8);
    RowVectorXd epsyblock(1, 8);
    RowVectorXd epszblock(1, 8);
    VectorXd q0(8);
    VectorXd d(8);
    VectorXd A(8);
    VectorXd a(8);
    MatrixXd Tf(4, 4);
    MatrixXd Th(4, 4);
    MatrixXd eps(3, 8);
    MatrixXd TM(4 * 8, 4);
    MatrixXd R(3, 3);
    Vector3d z0; Vector3d z1; Vector3d z2; Vector3d z3; Vector3d z4; Vector3d z5; Vector3d z6; Vector3d z7; Vector3d z8;
    Vector3d O1; Vector3d O2; Vector3d O3; Vector3d O4; Vector3d O5; Vector3d O6; Vector3d O7; Vector3d O8;
    MatrixXd R1(3, 3); MatrixXd R2(3, 3); MatrixXd R3(3, 3); MatrixXd R4(3, 3); MatrixXd R5(3, 3); MatrixXd R6(3, 3); MatrixXd R7(3, 3); MatrixXd R8(3, 3);
    MatrixXd J8(6, 8); MatrixXd Jaclas(3, 8);
    VectorXd z3xO8O2(3, 1);
    VectorXd z4xO8O3(3, 1);
    VectorXd z5xO8O4(3, 1);
    VectorXd z6xO8O5(3, 1);
    VectorXd z7xO8O6(3, 1);
    VectorXd z8xO8O7(3, 1);
    VectorXd zeros31(3, 1);
    RowVectorXd zeros116(1, 16);
    zeros116.setZero();
    RowVectorXd zeros18(1, 8);
    zeros18.setZero();
    MatrixXd z(3,3);
    MatrixXd Kc(3,3);
    double NormepsX = 0.0;
    double NormepsY = 0.0;
    double NormepsZ = 0.0;
    double zz, zzz, normresta;
    
    VectorXd E(3), Ef(3), u(3), JJxTc(8), JJyTc(8), JJzTc(8);
    MatrixXd eta(3, 3), K(3, 3), Kf(3,3), Identity8(8, 8);
    int warn = 0;
    MatrixXd JAestu1prev(3,8);
    JAestu1prev=JAest;

    eta << 0.1, 0, 0,
        0, 0.1, 0,
        0, 0, 0.1;

    K << 0, 0, 0,
        0, 0.4, 0,
        0, 0, 0.4;

    Kf << 0.1, 0, 0,
        0, 0, 0,
        0, 0, 0;

    double normw_2 = 0.0;
    //double t = 0;
    const double pi = M_PI;

    // %= ==================================================================================================== %
    // %                             Initialization ANN parameters for the SYSTEM
    // %= ==================================================================================================== %

    double B1_cn1 = 7;
    double B2_cn1 = 5;
    double B3_cn1 = 3.5;
    double B4_cn1 = 2;
    double B5_cn1 = 1;

    //B_cn1<<B1_cn1, B2_cn1, B3_cn1, B4_cn1, B5_cn1;

    double B1_cn2 = 7;
    double B2_cn2 = 5;
    double B3_cn2 = 3.5;
    double B4_cn2 = 2;
    double B5_cn2 = 1;

    //B_cn2<<B1_cn2, B2_cn2, B3_cn2, B4_cn2, B5_cn2;

    // ------Positive large
    double a_pl_cn1 = .845; double b_pl_cn1 = 45.0;
    // ------Positive small
    double a_ps_cn1 = .73; double b_ps_cn1 = 0.14;
    // ------Zero
    double a_ze_cn1 = .5; double b_ze_cn1 = 0.14;
    // ------Negative small
    double a_ns_cn1 = .27; double b_ns_cn1 = 0.14;
    // ------Negative large
    double a_nl_cn1 = -0.155; double b_nl_cn1 = 45.0;

    // ------Positive large
    double a_pl_cn2 = .695; double b_pl_cn2 = 60;
    // ------Positive small
    double a_ps_cn2 = .6; double b_ps_cn2 = 0.125;
    // ------Zero
    double a_ze_cn2 = .4; double b_ze_cn2 = 0.125;
    // ------Negative small
    double a_ns_cn2 = .2; double b_ns_cn2 = 0.125;
    // ------Negative large
    double a_nl_cn2 = -0.103; double b_nl_cn2 = 60.0;

    VectorXd o_opt(3);

    // --------------DH Parameters---------------------------------------- - %

    q0 << pi / 2, pi / 2, qu1(2) - pi / 2, qu1(3), qu1(4) - pi / 2, qu1(5), qu1(6) + pi / 2, qu1(7);

    d << qu1(0), qu1(1), 0, -.161, 0, 0, 0, 0.113;
    A << 0, -0.084, 0, 0.143, 0, 0.155, 0.135, 0;

    a << -pi / 2, pi / 2, -pi / 2, pi, pi / 2, 0, 0, pi / 2;

    Tf.setIdentity();
cout<<"---------------------------------"<<endl;
    for (int iii = 0; iii < 8; iii++)
    {
        Th << cos(q0(iii)), -sin(q0(iii)), 0, A(iii),
            cos(a(iii))* sin(q0(iii)), cos(q0(iii))* cos(a(iii)), -sin(a(iii)), -d(iii) * sin(a(iii)),
            sin(a(iii))* sin(q0(iii)), sin(a(iii))* cos(q0(iii)), cos(a(iii)), d(iii)* cos(a(iii)),
            0, 0, 0, 1;

        Tf = Tf * Th;
        TM.block(4 * iii, 0, 4, 4) = Tf;

        //cout<<TM.block(4 * (i - 1), 0, 4, 4)<<endl;
    }

    z1 = TM.block(0, 2, 3, 1);
    z2 = TM.block(4, 2, 3, 1);
    z3 = TM.block(8, 2, 3, 1);
    z4 = TM.block(12, 2, 3, 1);
    z5 = TM.block(16, 2, 3, 1);
    z6 = TM.block(20, 2, 3, 1);
    z7 = TM.block(24, 2, 3, 1);
    z8 = TM.block(28, 2, 3, 1);

    O1 = TM.block(0, 3, 3, 1);
    O2 = TM.block(4, 3, 3, 1);
    O3 = TM.block(8, 3, 3, 1);
    O4 = TM.block(12, 3, 3, 1);
    O5 = TM.block(16, 3, 3, 1);
    O6 = TM.block(20, 3, 3, 1);
    O7 = TM.block(24, 3, 3, 1);
    O8 = TM.block(28, 3, 3, 1);

    R1 = TM.block(0, 0, 3, 3);
    R2 = TM.block(4, 0, 3, 3);
    R3 = TM.block(8, 0, 3, 3);
    R4 = TM.block(12, 0, 3, 3);
    R5 = TM.block(16, 0, 3, 3);
    R6 = TM.block(20, 0, 3, 3);
    R7 = TM.block(24, 0, 3, 3);
    R8 = TM.block(28, 0, 3, 3);

    //%% ------------------ - GJ computation------------------------ - %

    z3xO8O2 = z3.cross(O8 - O2);
    z4xO8O3 = z4.cross(O8 - O3);
    z5xO8O4 = z5.cross(O8 - O4);
    z6xO8O5 = z6.cross(O8 - O5);
    z7xO8O6 = z7.cross(O8 - O6);
    z8xO8O7 = z8.cross(O8 - O7);

    J8 << z1, z2, z3xO8O2, z4xO8O3, z5xO8O4, z6xO8O5, z7xO8O6, z8xO8O7,
          zeros31, zeros31, z3, z4, z5, z6, z7, z8;
    
    VectorXd o_opt(3);


    VectorXd w(8);
    w.setZero();
    w=dq;

    Identity8.setIdentity();
    u.setZero();
    VectorXd alldata(42);
    double mu = 1.0;
    MatrixXd pinvJest(8, 3);

    //VectorXd p(3 * kmax);
    VectorXd out_error(3);

    //ef.setLinSpaced(point, e_min, e_max);

    //double ex, ey, ez;

    out_error = v - JAest * w;

    normw_2 = w.norm() * w.norm();
    
    Jaclas = J8.block(0,0,3,8);
    eps = Jaclas - JAest;
    
    NormepsX = eps.block(0, 0, 1, 8).norm();
    NormepsY = eps.block(1, 0, 1, 8).norm();
    NormepsZ = eps.block(2, 0, 1, 8).norm();
    //----------------------------------------------------------------------------------//
        //----------------------Network with estimation error as input----------------------//
        //----------------------------------------------------------------------------------//

        //= ====================================== X ========================================== = $
    
    M1PLX = MFsig01(NormepsX, a_pl_cn2, b_pl_cn1);
    M1PSX = MFgus01(NormepsX, a_ps_cn2, b_ps_cn1);
    M1ZEX = MFgus01(NormepsX, a_ze_cn2, b_ze_cn1);
    M1NSX = MFgus01(NormepsX, a_ns_cn2, b_ns_cn1);
    M1NLX = MFsig01n(NormepsX, a_nl_cn2, b_nl_cn1);

    double C_1X = B1_cn1 * M1PLX + B2_cn1 * M1PSX + B3_cn1 * M1ZEX + B4_cn1 * M1NSX + B5_cn1 * M1NLX;

    M2PLX = MFsig01(NormepsX, a_pl_cn2, b_pl_cn2);
    M2PSX = MFgus01(NormepsX, a_ps_cn2, b_ps_cn2);
    M2ZEX = MFgus01(NormepsX, a_ze_cn2, b_ze_cn2);
    M2NSX = MFgus01(NormepsX, a_ns_cn2, b_ns_cn2);
    M2NLX = MFsig01n(NormepsX, a_nl_cn2, b_nl_cn2);

    double C_2X = B1_cn2 * M2PLX + B2_cn2 * M2PSX + B3_cn2 * M2ZEX + B4_cn2 * M2NSX + B5_cn2 * M2NLX;

    //= ====================================== Y ========================================== = $

    M1PLY = MFsig01(NormepsY, a_pl_cn2, b_pl_cn1);
    M1PSY = MFgus01(NormepsY, a_ps_cn2, b_ps_cn1);
    M1ZEY = MFgus01(NormepsY, a_ze_cn2, b_ze_cn1);
    M1NSY = MFgus01(NormepsY, a_ns_cn2, b_ns_cn1);
    M1NLY = MFsig01n(NormepsY, a_nl_cn2, b_nl_cn1);

    double C_1Y = B1_cn1 * M1PLY + B2_cn1 * M1PSY + B3_cn1 * M1ZEY + B4_cn1 * M1NSY + B5_cn1 * M1NLY;

    M2PLY = MFsig01(NormepsY, a_pl_cn2, b_pl_cn2);
    M2PSY = MFgus01(NormepsY, a_ps_cn2, b_ps_cn2);
    M2ZEY = MFgus01(NormepsY, a_ze_cn2, b_ze_cn2);
    M2NSY = MFgus01(NormepsY, a_ns_cn2, b_ns_cn2);
    M2NLY = MFsig01n(NormepsY, a_nl_cn2, b_nl_cn2);

    double C_2Y = B1_cn2 * M2PLY + B2_cn2 * M2PSY + B3_cn2 * M2ZEY + B4_cn2 * M2NSY + B5_cn2 * M2NLY;

    //= ====================================== Z ========================================== = $

    M1PLZ = MFsig01(NormepsZ, a_pl_cn2, b_pl_cn1);
    M1PSZ = MFgus01(NormepsZ, a_ps_cn2, b_ps_cn1);
    M1ZEZ = MFgus01(NormepsZ, a_ze_cn2, b_ze_cn1);
    M1NSZ = MFgus01(NormepsZ, a_ns_cn2, b_ns_cn1);
    M1NLZ = MFsig01n(NormepsZ, a_nl_cn2, b_nl_cn1);

    double C_1Z = B1_cn1 * M1PLZ + B2_cn1 * M1PSZ + B3_cn1 * M1ZEZ + B4_cn1 * M1NSZ + B5_cn1 * M1NLZ;

    M2PLZ = MFsig01(NormepsZ, a_pl_cn2, b_pl_cn2);
    M2PSZ = MFgus01(NormepsZ, a_ps_cn2, b_ps_cn2);
    M2ZEZ = MFgus01(NormepsZ, a_ze_cn2, b_ze_cn2);
    M2NSZ = MFgus01(NormepsZ, a_ns_cn2, b_ns_cn2);
    M2NLZ = MFsig01n(NormepsZ, a_nl_cn2, b_nl_cn2);

    double C_2Z = B1_cn2 * M2PLZ + B2_cn2 * M2PSZ + B3_cn2 * M2ZEZ + B4_cn2 * M2NSZ + B5_cn2 * M2NLZ;
    
    RowVectorXd JJx(1, 8); JJx = JAestu1.block(0,0,1,8);
    RowVectorXd JJy(1, 8); JJy = JAestu1.block(1,0,1,8);
    RowVectorXd JJz(1, 8); JJz = JAestu1.block(2,0,1,8);

    JJxT = JJx.transpose();
    JJyT = JJy.transpose();
    JJzT = JJz.transpose();
    
    double aaJJx; aaJJx = JJx * JJxT;
    double aaJJy; aaJJy = JJy * JJyT;
    double aaJJz; aaJJz = JJz * JJzT;

    double Ckx = C_1X / (rho1 + C_1X * C_1X * aaJJx);

    double Cky = C_1Y / (rho2 + C_1Y * C_1Y * aaJJy);

    double Ckz = C_1Z / (rho3 + C_1Z * C_1Z * aaJJz);

    CKK << Ckx, 0, 0,
           0, Cky, 0,
           0, 0, Ckz;

    MatrixXd C_1(3, 3); C_1 << C_1X, 0, 0,
                               0, C_1Y, 0,
                               0, 0, C_1Z;

    MatrixXd C_2(3, 3); C_2 << C_2X, 0, 0,
                               0, C_2Y, 0,
                               0, 0, C_2Z;

    cout<<"C_1= "<<C_1X<<", "<<C_1Y<<", "<<C_1Z<<endl;
    cout<<"C_2= "<<C_2X<<", "<<C_2Y<<", "<<C_2Z<<endl;
    
    JAest = JAest + (eta * (out_error)) * w.transpose() / (mu + normw_2);

    pinvJest = (JAest.transpose() * JAest + Identity8 * 0.01).inverse() * JAest.transpose();
    
    z = Jaclas * pinvJest;
    zz = z.norm();
    normresta=1.0 - zz;
    zzz = -normresta;
    
    cout<<"Error de estimación= "<<zzz<<endl;

    E << 0, e(1), e(2);

    Ef << ef1, 0, 0;
    
    std::cout << std::endl << "Errores" << std::endl;
    std::cout << "+-----------------------------------------" << std::endl
        << "|  Error" << "\t" << "Valor" << std::endl
        << "|  ex" << "\t\t" << E(0) << std::endl
        << "|  ey" << "\t\t" << E(1) << std::endl
        << "|  ez" << "\t\t" << E(2) << std::endl
        << "+-----------------------------------------" << std::endl;
    
    pu1(0)=0; XDpp(0)=0;	
    u = C_1 * (pu1 - XDpp) + (C_2 - C_1) * E + Kf * Ef;
	
    /*u = K * E + Kf * Ef;*/

    w = -pinvJest * u;

    for (int i = 0; i < 8; i++) {

        if (w(i) >= 0.5) {
            w(i) = 0.5;
        }
        if (w(i) <= -0.5) {
            w(i) = -0.5;
        }
    }
    JJxTc = JAest.block(0, 0, 1, 8).transpose();
    JJyTc = JAest.block(1, 0, 1, 8).transpose();
    JJzTc = JAest.block(2, 0, 1, 8).transpose();

    alldata << w, JJxTc, JJyTc, JJzTc, zzz, C_1X, C_1Y, C_1Z, C_2X, C_2Y, C_2Z, u(0), u(1), u(2);

    return alldata;

}

void communication_robot1(int sockfd)
{
    //     KukaYb Youbot_ctrl;
    data_package data_rob_r;
    data_package data_rob_s;
    ServerComm servercomm;

    VectorXd ctrl_result(23), q(8), dq(8), JJxT1(8), JJyT1(8), JJzT1(8);
    VectorXd p1_temp(3), v1(3), e11(3), X_11dpp(3);
    double ef=0.0;
    double f1=0.0;
    ctrl_result.setZero();
    dq.setZero();

    MatrixXd JAest(3, 8);
    JAest << 0, 1, 0, 0, .06, .02, 0, .06,
        1, 0, 0, 0, .01, -.04, 0, .03,
        0, 0, 0, 0, -0.05, -0.03, 0, 0;

    double t = 0;
    double dt = 0.01;

    ros::Rate loop_rate(20);

    auto t_init = std::chrono::high_resolution_clock::now();

    while (ros::ok())
    {
        auto t_start = std::chrono::high_resolution_clock::now();

        optitrack_readings();
        r11_coord_transformation();

        cout << "xyz r1 = " << X_11i(0) << "\t" << X_11i(1) << "\t" << X_11i(2) << endl << endl;
        cout << "xyz o1= " << X_11d(0) << "\t" << X_11d(1) << "\t" << X_11d(2) << endl << endl;

        data_rob_r = servercomm.data_receptor(sockfd);

        q(0) = data_rob_r.q1;
        q(1) = data_rob_r.q2;
        q(2) = data_rob_r.q3;
        q(3) = data_rob_r.q4;
        q(4) = data_rob_r.q5;
        q(5) = data_rob_r.q6;
        q(6) = data_rob_r.q7;
        q(7) = data_rob_r.q8;
	f1 = data_rob_r.f;
	
	cout << "Fuerza recibida: " << f1 << endl;

        v1 = (X_11i - p1_temp) * (1 / dt);
	
	ef = f1 - 0.6;

	X_11d(2) = X_11d(2) - .02;
	
        e11 = X_11i - X_11d;

        p1_temp = X_11i;
	
	X_11dpp = X_11d;

            // Agregar aqui el código para estimación, adaptación y control del KUKA 1
        ctrl_result = Control_Kuka(e11, t, dq, JAest, v1, ef, q, X_11i, X_11dpp);

        dq = ctrl_result.block(0, 0, 8, 1);

        JJxT1 = ctrl_result.block(8, 0, 8, 1); JJyT1 = ctrl_result.block(16, 0, 8, 1); JJzT1 = ctrl_result.block(24, 0, 8, 1);

        JAest << JJxT1.transpose(), JJyT1.transpose(), JJzT1.transpose();

	double est_error1; est_error1 = ctrl_result(32);
	VectorXd a11(3); a11 = ctrl_result.block(33, 0, 3, 1);
	VectorXd a12(3); a12 = ctrl_result.block(36, 0, 3, 1);
	VectorXd control1(3); control1 = ctrl_result.block(39, 0, 3, 1);
	
        data_rob_s.dq1 = dq(0);
        data_rob_s.dq2 = dq(1);
        data_rob_s.dq3 = dq(2);
        data_rob_s.dq4 = dq(3);
        data_rob_s.dq5 = dq(4);
        data_rob_s.dq6 = dq(5);
        data_rob_s.dq7 = dq(6);
        data_rob_s.dq8 = dq(7);
	data_rob_s.t = t;
	data_rob_s.esterror = est_error1;
	data_rob_s.a1x = a11(0);
	data_rob_s.a1y = a11(1);
	data_rob_s.a1z = a11(2);
	data_rob_s.a2x = a12(0);
	data_rob_s.a2y = a12(1);
	data_rob_s.a2z = a12(2);
	data_rob_s.ux = control1(0);
	data_rob_s.uy = control1(1);
	data_rob_s.uz = control1(2);
	data_rob_s.x = X_11i(0);
	data_rob_s.y = X_11i(1);
	data_rob_s.z = X_11i(2);
	data_rob_s.xr = X_11d(0);
	data_rob_s.yr = X_11d(1);
	data_rob_s.zr = X_11d(2);
	data_rob_s.xrpp = 0;
	data_rob_s.yrpp = 0;
	data_rob_s.zrpp = 0;
	 
        // 	data_rob_s.dq1 = 0;
        //         data_rob_s.dq2 = 0;
        //         data_rob_s.dq3 = 0;
        //         data_rob_s.dq4 = 0;
        //         data_rob_s.dq5 = 0;
        //         data_rob_s.dq6 = 0;
        //         data_rob_s.dq7 = 0;
        //         data_rob_s.dq8 = 0;

        servercomm.data_transmission(sockfd, data_rob_s);

        auto t_end = std::chrono::high_resolution_clock::now();

        cout << std::chrono::duration<double>(t_end - t_start).count()
            << " s\n" << endl
            << "current time = " << std::chrono::duration<double>(t_end - t_init).count() << endl << endl;
        t = std::chrono::duration<double>(t_end - t_init).count();

        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "KukaYoubot");
    ros::NodeHandle n("~"), n1, n2, n3, n4, n5, n6, n7, n8, n9;
    ros::Subscriber sub1 = n1.subscribe("vrpn_client_node/Pose1/pose", 1000, chatterCallback);
    ros::Subscriber sub2 = n2.subscribe("vrpn_client_node/Base1/pose", 1000, chatterCallback2);
    ros::Subscriber sub3 = n3.subscribe("vrpn_client_node/Object1/pose", 1000, chatterCallback3);
    ros::Subscriber sub4 = n4.subscribe("vrpn_client_node/Irobot/pose", 1000, chatterCallback4);
    ros::Subscriber sub5 = n5.subscribe("vrpn_client_node/WoodBase/pose", 1000, chatterCallback5);
    ros::Subscriber sub6 = n6.subscribe("vrpn_client_node/Base2/pose", 1000, chatterCallback6);
    ros::Subscriber sub7 = n7.subscribe("vrpn_client_node/Pose2/pose", 1000, chatterCallback7);
    ros::Subscriber sub8 = n8.subscribe("vrpn_client_node/BubblesBase/pose", 1000, chatterCallback8);
    ros::Subscriber sub9 = n9.subscribe("vrpn_client_node/BoxBase/pose", 1000, chatterCallback9);

    ServerComm servercomm;
    int sock1fd;
    //float data;

    X_11i.setZero();
    ros::Rate loop_rate(20);

    // Primer loop para la inicialización

    while (ros::ok() && ((X_11i.norm() <= 0)))
    {
        optitrack_readings();
        r11_coord_transformation();
        cout << "getting coordinates..." << endl << endl;
        ros::spinOnce();
        loop_rate.sleep();
    }

    //Se imprimen los valores iniciales

    cout << "R1 pos =" << X_11i(0) << "\t" << X_11i(1) << "\t" << X_11i(2) << endl << endl;

    sock1fd = servercomm.socket_server();
    cout << "Communication stablished" << endl;

    // Se definen las estructuras para la comunicación por sockets
    data_package data_rob1_r;
    data_package data_rob1_s;

    VectorXd qR1(8);
    float f=0.0;

    // Robot 1
    // Coordenadas locales iniciales de la base móvil
    qR1(0) = 0;
    qR1(1) = 0;
    qR1(2) = 0;

    // Coordenadas iniciales del brazo
    qR1(3) = 0;
    qR1(4) = 0;
    qR1(5) = 0;
    qR1(6) = 0;
    qR1(7) = 0;

    // Envío de coordenadas articulares iniciales deseadas
    data_rob1_s.q1 = qR1(0);
    data_rob1_s.q2 = qR1(1);
    data_rob1_s.q3 = qR1(2);
    data_rob1_s.q4 = qR1(3);
    data_rob1_s.q5 = qR1(4);
    data_rob1_s.q6 = qR1(5);
    data_rob1_s.q7 = qR1(6);
    data_rob1_s.q8 = qR1(7);

    // Envío de datos
    servercomm.data_transmission(sock1fd, data_rob1_s);

    cout << "waiting client msg" << endl;
    // Recepción de datos
    data_rob1_r = servercomm.data_receptor(sock1fd);

    //Coordenadas actuales sensadas
    qR1(0) = data_rob1_r.q1;
    qR1(1) = data_rob1_r.q2;
    qR1(2) = data_rob1_r.q3;
    qR1(3) = data_rob1_r.q4;
    qR1(4) = data_rob1_r.q5;
    qR1(5) = data_rob1_r.q6;
    qR1(6) = data_rob1_r.q7;
    qR1(7) = data_rob1_r.q8;
    f = data_rob1_r.f;

    thread* robot1;

    robot1 = new thread(communication_robot1, sock1fd);

    robot1->join();

    delete robot1;

    ros::spin();
    ros::shutdown();

    return 0;
}