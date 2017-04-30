/* +---------------------------------------------------------------------------+
  /*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   wheel_odometry.hpp
 * Author: Dibyendu Ghosh
 * Organization : Intel Research Lab bangalore
 * Created on 12 February, 2016, 11:50 AM
 */
// ------------------------------------------------------*/
#include<iostream>
#include <mrpt/poses.h>

#ifndef WHEEL_ODOMETRY_HPP
#define WHEEL_ODOMETRY_HPP
class Wheel_Odom
{
public:
	mrpt::poses::CPose2D wheel_odometry(double left_EncoderCount, double right_EncoderCount /*double imu_yaw*/);
};


#endif /* Wheel_Odometry_HPP */
