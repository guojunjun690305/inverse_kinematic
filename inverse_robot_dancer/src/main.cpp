#include <iostream>
#include <time.h>
#include <unistd.h>
#include <vector>
#include <utility>
#include <sstream>
#include <math.h>
#include <string>
#include <boost/numeric/ublas/matrix.hpp>
//#include <algorithm>
//#include <cmath>

//#include "port_handler.h"
//#include<ros/ros.h>
//#include<std_msgs/String.h>
//#include"std_msgs/String.h"

using namespace std;

double hip_distance = 0.16;
double upper_leg = 0.35;
double lower_leg = 0.35;
double upper_arm = 0.35;
double lower_arm = 0.35;

//double lbz_distance = 0.14;//0
//double g = 980;//980
//double dt = 0.012;//0.012

string supportStatus = "SINGLE_BASED";//DOUBLE_BASED   SINGLE_BASED
double yzmp = 0;

typedef boost::numeric::ublas::matrix<double> boost_matrix;


boost_matrix rotx(double roll)
{
    boost_matrix Rx(3, 3);
    Rx(0, 0) = 1;
    Rx(0, 1) = 0;
    Rx(0, 2) = 0;
    Rx(1, 0) = 0;
    Rx(1, 1) = cos(roll);
    Rx(1, 2) = -sin(roll);
    Rx(2, 0) = 0;
    Rx(2, 1) = sin(roll);
    Rx(2, 2) = cos(roll);
    return Rx;
}

boost_matrix roty(double pit)
{
    boost_matrix Ry(3, 3);
    Ry(0, 0) = cos(pit);
    Ry(0, 1) = 0;
    Ry(0, 2) = sin(pit);
    Ry(1, 0) = 0;
    Ry(1, 1) = 1;
    Ry(1, 2) = 0;
    Ry(2, 0) = -sin(pit);
    Ry(2, 1) = 0;
    Ry(2, 2) = cos(pit);
    return Ry;
}

boost_matrix rotz(double yaw)
{
    boost_matrix Rz(3, 3);
    Rz(0, 0) = cos(yaw);
    Rz(0, 1) = -sin(yaw);
    Rz(0, 2) = 0;
    Rz(1, 0) = sin(yaw);
    Rz(1, 1) = cos(yaw);
    Rz(1, 2) = 0;
    Rz(2, 0) = 0;
    Rz(2, 1) = 0;
    Rz(2, 2) = 1;
    return Rz;
}


boost_matrix rpy2r(double roll, double pit, double yaw) /// ypr order in gyroscope
{
    //旋转的顺序是 yaw , pitch , roll
    boost_matrix R_tmp(3, 3);
    R_tmp = prod(rotz(yaw), roty(pit));
    return prod(R_tmp, rotx(roll));
}


boost_matrix Array2row(double* t_array, int row)
{
    boost_matrix R_tmp((unsigned long)row, 1);
    for (int i = 0; i < row; i++)
        R_tmp((unsigned long)i, 0) = t_array[i];
    return R_tmp;
}

double* get_Angle(const double* tChest, const double* tAnkle, const double* tHand, const bool whleg)
{
    double fChest_P[3], fChest_RPY[3], fAnkle_P[3], fAnkle_RPY[3];
    for (int i = 0; i < 3; i++) {
        fAnkle_P[i] = tAnkle[i];
        fChest_P[i] = tChest[i];
        fChest_RPY[i] = tChest[i + 3];
        fAnkle_RPY[i] = tAnkle[i + 3];
    }

    /// ROBOT CONFIG
    double l1 = - upper_leg;
    double l2 = - lower_leg;
    double l3 =  upper_arm;
    double l4 =  lower_arm;
    double lby, lbz;//质心和大腿心的距离
    double distance_com2hip = 0.1;
    lbz = - distance_com2hip;

    // FIXME fucking yzmp
    if (supportStatus == "DOUBLE_BASED") {
        if (whleg == 0) // 0 means rightLeg
            lby = - hip_distance / 2 -  yzmp; //理解时可以先将yzmp 假想成0
        else
            lby =  hip_distance / 2 +  yzmp;
    }
    else {
        if (whleg == 0) // 0 means rightLeg
            lby = - hip_distance / 2 -  yzmp;
        else
            lby =  hip_distance / 2 +  yzmp;
    }

    boost_matrix Lb(3, 1);
    Lb(0, 0) = 0;
    Lb(1, 0) = lby;
    Lb(2, 0) = lbz;

    double* q = new double[9];
    // (FIXME): MWX memory leak!!!!!!!!!!!!!!!!!!
    for (int i = 0; i < 9; i++)
        q[i] = 0;
    //    boost_matrix Chest_R, Ankle_R, Chest_P, Ankle_P, Hip_P, r;
    auto Chest_R = rpy2r(fChest_RPY[0], fChest_RPY[1], fChest_RPY[2]);
    auto Chest_P = Array2row(fChest_P, 3);
    auto Ankle_R = rpy2r(fAnkle_RPY[0], fAnkle_RPY[1], fAnkle_RPY[2]);
    auto Ankle_P = Array2row(fAnkle_P, 3);
    auto Hip_P = Chest_P + prod(Chest_R, Lb);

    auto r = prod(trans(Ankle_R), Hip_P - Ankle_P);
    double D = sqrt(pow(r(0, 0), 2) + pow(r(1, 0), 2) + pow(r(2, 0), 2));
    double tmp_q3 = (D * D - l1 * l1 - l2 * l2) / (2 * l1 * l2);
    if (tmp_q3 >= 1)
        q[3 - 1] = 0;
    else if (tmp_q3 <= -1)
        q[3 - 1] = M_PI;
    else
        q[3 - 1] = acos(tmp_q3);
    double q6a = asin(l2 * sin(M_PI - q[3 - 1]) / D);
    q[2 - 1] = -atan2(r(0, 0), (r(2, 0) / abs(r(2, 0))) * sqrt(r(1, 0) * r(1, 0) + r(2, 0) * r(2, 0))) + q6a;
    q[1 - 1] = atan2(r(1, 0), r(2, 0));

    if (q[1 - 1] > 0.5 * M_PI)
        q[1 - 1] = q[1 - 1] - M_PI;
    else if (q[1 - 1] < -0.5 * M_PI)
        q[1 - 1] = q[1 - 1] + M_PI;

    boost_matrix tmp1, tmp2, RR;
    tmp1 = prod(trans(Chest_R), Ankle_R);
    tmp2 = prod(tmp1, rotx(-q[1 - 1]));
    RR = prod(tmp2, roty(-q[3 - 1] - q[2 - 1]));

    q[6 - 1] = atan2(-RR(0, 1), RR(1, 1)); //股关节yaw角
    q[5 - 1] = atan2(RR(2, 1), -RR(0, 1) * sin(q[6 - 1]) + RR(1, 1) * cos(q[6 - 1])); // 股关节roll角
    q[4 - 1] = atan2(-RR(2, 0), RR(2, 2)); //股关节pitch角

    /// for hand id 1 17 18(right hand) id 8 15 16(left
    /// hand)(TODO:解算手部逆运动学需重新推倒，
    //当前只是简单的平面二连杆解析解)
    q[7 - 1] = 0;
    q[8 - 1] = 0;
    q[9 - 1] = 0;
    double x, z;
    double thand0 = tHand[0];
    if (thand0 >=  lower_arm +  upper_arm)
        thand0 =  lower_arm +  upper_arm;

    x = tHand[0] * sin(tHand[1] * M_PI / 180);
    z = -tHand[0] * cos(tHand[1] * M_PI / 180);
    double theta2, theta4, theta1, theta3;

    theta2 = M_PI - acos((l3 * l3 + l4 * l4 - x * x - z * z) / 2 / l3 / l4);

    theta4 = acos(-z / (sqrt(x * x + z * z)));
    if (x < 0) {
        theta4 = -theta4;
    }

    theta3 = acos((z * z + x * x + l3 * l3 - l4 * l4) / 2 / l3 / sqrt(x * x + z * z));

    theta1 = theta4 - theta3;
    // TODO::when there is a situation the current theta1 angle is 30degree and
    // next is -30
    // the motor will move 30->350 not 30->-30
    // template annotate it to solve
    // ***********
    // if (theta1 > M_PI) theta1 = theta1 - 2 * M_PI;
    // if (theta1 < -M_PI) theta1 = theta1 + 2 * M_PI;
    //**********
    q[7 - 1] = theta1;
    q[9 - 1] = theta2;
    return q;
}



int main()
{
/************************************************
 *  robot functions add by davince 2014.10.9
 * \brief get_Angle
 * inverse kinetics
 * \param tChest Chest Postion (x,y,z,r,p,y)
 * \param tAnkle Ankle Position (x,y,z,r,p,y)
 * \param tHand Hand Position (x,y,z,r,p,y)
 * \param whleg left or right leg
 * \return *q 6x1 legs' angle (rad)
 *详细可见 《仿人机器人第2章》逆运动学
 ********************************
 郭军军 获得各个关节角度的值
 ***********************************************/
    double* rq, * lq;
    /// IK function
    double H_cm[6], H_ra[6], H_rh[6], H_la[6], H_lh[6];
    for(int i=0;i<6;i++)
    {
        H_cm[i] = 0;
        H_ra[i] = 0;
        H_rh[i] = 0;
        H_la[i] = 0;
        H_lh[i] = 0;
    }

    H_ra[0] = 0.23; H_ra[1] = -0.08; H_ra[2] = -0.6;
    H_la[0] = 0.23; H_la[1] = 0.08 ; H_la[2] = -0.6;

    rq = get_Angle(H_cm, H_ra, H_rh, 0);
    lq = get_Angle(H_cm, H_la, H_lh, 1);

    for(int i=0;i<6;i++)
    {
        std::cout<<"rq"<<i<<" = "<<rq[i]<<std::endl;
    }
     for(int i=0;i<6;i++)
    {
        std::cout<<"lq"<<i<<" = "<<lq[i]<<std::endl;
    }
    
    float joint_values[18];
    joint_values[6] = rq[0];//踝关节roll角
    joint_values[5] = rq[1];//踝关节pitch角  弯曲时计算为负值
    joint_values[4] = rq[2];//膝关节pitch角  弯曲时计算为正值
    joint_values[1] = rq[3];//股关节pitch角  弯曲时计算为负值
    joint_values[2] = rq[4];//股关节roll角
    joint_values[3] = rq[5];//股关节yaw角
    joint_values[0] = rq[6];
    joint_values[16] = rq[7];
    joint_values[17] = rq[8];

    joint_values[13] = lq[0];
    joint_values[12] = lq[1];
    joint_values[11] = lq[2];
    joint_values[8]  = lq[3];
    joint_values[9]  = lq[4];
    joint_values[10] = lq[5];
    joint_values[7]  = lq[6];
    joint_values[14] = lq[7];
    joint_values[15] = lq[8];

    float left_knee_pitch  = lq[2];
    float left_hip_yaw     = lq[5];
    float left_hip_pitch   = lq[3];
    float left_hip_roll    = lq[4];
    float left_ankle_pitch = lq[1];
    float left_ankle_roll  = lq[0];

    float right_knee_pitch  = rq[2];
    float right_hip_yaw     = rq[5];
    float right_hip_pitch   = rq[3];
    float right_hip_roll    = rq[4];
    float right_ankle_pitch = rq[1];
    float right_ankle_roll  = rq[0];

    return 1;

}
