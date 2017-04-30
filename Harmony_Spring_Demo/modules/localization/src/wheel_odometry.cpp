/* +---------------------------------------------------------------------------+
 /*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   wheel_odometry.cpp
 * Author: Dibyendu Ghosh
 * Organization : Intel Research Lab bangalore
 * Created on 12 February, 2016, 11:50 AM
 */
// ------------------------------------------------------*/
#include "../include/wheel_odometry.hpp"
#include <iostream>

#define wheel_base 0.485 /*wheel base of the robot in m */
#define left_count_per_cm 45 /* left wheel encoder count in one cm*/
#define right_count_per_cm 45 /* left wheel encoder count in one cm*/

#define left_wheel_radius 0.315  /* left wheel radius m*/
#define right_wheel_radius 0.315 /* left wheel radius m*/
#define wheel_diameter 0.6
#define gear_ratio 1/1
#define encoder_resolution 1080

#define scale_factor (M_PI* wheel_diameter)/(gear_ratio*encoder_resolution)          //conversion factor that translates encoder pulses into linear wheel displacement

/*wheel odometry Kinematic model for differential drive robot*/


double odom_x, odom_y, odom_yaw;
mrpt::poses::CPose2D Wheel_Odom::wheel_odometry(double left_EncoderCount,double right_EncoderCount) {

	left_EncoderCount = left_EncoderCount * scale_factor; //displacement
	right_EncoderCount = left_EncoderCount * scale_factor; //displacement
	double ang_velocity = (1 / wheel_base)* (right_EncoderCount - left_EncoderCount); //
	double displacement = 0.5 * (right_EncoderCount + left_EncoderCount);  //
	odom_yaw += ang_velocity;
	odom_x += displacement * cos(odom_yaw);
	odom_y += displacement * sin(odom_yaw);
	return mrpt::poses::CPose2D(odom_x,odom_y,odom_yaw);
}
