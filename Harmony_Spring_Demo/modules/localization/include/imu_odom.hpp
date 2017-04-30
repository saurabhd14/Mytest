/* +---------------------------------------------------------------------------+
  /*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   imu_odom.hpp
 * Author: Dibyendu Ghosh
 * Organization : Intel Research Lab bangalore
 * Created on 12 February, 2016, 11:50 AM
 */
// ------------------------------------------------------*/
#include<iostream>
#include <mrpt/poses.h>

#ifndef IMU_ODOM_HPP
#define IMU_ODOM_HPP
class IMU_POSE
{
public:
	mrpt::poses::CPose3D imu_pose(double yaw, double pitch, double roll, double ax, double ay, double az,mrpt::poses::CPose3D currected_pose );
};

#endif /* IMU_Odometry_HPP */
