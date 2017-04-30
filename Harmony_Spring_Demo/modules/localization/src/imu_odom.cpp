/* +---------------------------------------------------------------------------+
  /*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   imu_odom.cpp
 * Author: Dibyendu Ghosh
 * Organization : Intel Research Lab bangalore
 * Created on 12 February, 2016, 11:50 AM
 */
// ------------------------------------------------------*/
#include "../include/imu_odom.hpp"
#include <iostream>
#include <mrpt/poses.h>

#define imux_bias 0.01  // calculate IMU X bias
#define imuy_bias 0.01  // calculate IMU Y bias
#define imuz_bias 0.01  // calculate IMU Y bias
#define halfT 1/30 //0.5f		// half the sample period of IMU data

double vel_x = 0, vel_y = 0, vel_z = 0;
double linear_x = 0.0, linear_y = 0.0, linear_z = 0.0;
double last_ax = 0.0, last_ay = 0.0, last_az = 0.0;

int count_x = 0.0, count_y = 0.0, count_z;
int imu_yaw = 0;
mrpt::poses::CPose3D last_pose;

mrpt::poses::CPose3D IMU_POSE::imu_pose(double yaw, double pitch, double roll, double ax, double ay, double az,mrpt::poses::CPose3D currected_pose ) {
double imu_x = 0, imu_y = 0, imu_z = 0;

    az = az - 9.81; //

    if (ax > imux_bias && fabs(last_ax - ax) < 0.5f) {
        vel_x += ax*halfT;
        //vel(t,:) = vel(t-1,:) + acc(t,:) * samplePeriod;
    } else
        ax = 0;

    if (ax == 0) {
        count_x++;
    } else
        count_x = 0;

    if (count_x > 20) {
        vel_x = 0;
    }
    last_ax = ax;


    if (ay > imuy_bias && fabs(last_ay - ay) < 0.5f) {
        vel_y += ay*halfT;
    } else
        ay = 0;
    if (ay == 0) {
        count_y++;
    } else
        count_y = 0;

    if (count_y > 20) {
        vel_y = 0;
    }
    last_ay = ay;



    if (az > imuy_bias && fabs(last_az - az) < 0.5f) {
        vel_z += az*halfT;
    } else
        az = 0;
    if (az == 0) {
        count_z++;
    } else
        count_z = 0;

    if (count_z > 15) {
        vel_z = 0;
    }
    last_az = az;

    linear_x = vel_x * halfT + 0.5 * ax * halfT*halfT; // check add part i.e linear_x++
    linear_y = vel_y * halfT + 0.5 * ay * halfT*halfT;
    linear_z = vel_z * halfT + 0.5 * az * halfT*halfT;

    //imu_x += linear_x * cos(yaw) + linear_y * sin(yaw); //current_p.x();
    //imu_y += -linear_x * sin(yaw) + linear_y * cos(yaw);//current_p.y();

    mrpt::poses::CPose3D rt_mat = mrpt::poses::CPose3D(0.0, 0.0, 0.0, yaw, pitch, roll);

    mrpt::math::CMatrixDouble31 lxyz;
    lxyz(0, 0) = linear_x;
    lxyz(1, 0) = linear_y;
    lxyz(2, 0) = linear_z;
    mrpt::math::CMatrixDouble31 rt_p = rt_mat.getRotationMatrix() * lxyz;
    imu_x =currected_pose.x()+rt_p(0, 0);
    imu_y =currected_pose.y()+rt_p(1, 0);
    imu_z =currected_pose.z()+ rt_p(2, 0);

    mrpt::poses::CPose3D p = mrpt::poses::CPose3D(imu_x, imu_y, imu_z, yaw, pitch, roll);
    //mrpt::math::CMatrixDouble44 imu_incr_pose = p.getHomogeneousMatrixVal();
    mrpt::poses::CPose3D imu_incr(p.x(), p.y(), p.z(), 0, 0, 0);
    mrpt::poses::CPose3D current_p;
    current_p.composeFrom(last_pose, imu_incr);


    last_pose = imu_incr; // = mrpt::poses::CPoint3D(imu_x, imu_y, imu_z, yaw, pitch, roll);

    return current_p;

}
