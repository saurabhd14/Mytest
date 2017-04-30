/* +---------------------------------------------------------------------------+
  /*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   uwb_imu.hpp
 * Author: Dibyendu Ghosh
 * Organization : Intel Research Lab bangalore
 * Created on 12 February, 2016, 11:50 AM
 */
// ------------------------------------------------------*/
#include <iostream>
#include <mrpt/utils.h>
#include <mrpt/math.h>
#include <mrpt/poses.h>
using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::utils;

#ifndef UWB_IMU_HPP
#define UWB_IMU_HPP
class uwb_imu {
public:
	mrpt::poses::CPose2D uwb_imu_position(double uwb_range, double yaw);
	std::pair<mrpt::math::TPoint2D,mrpt::math::TPoint2D> circle_intersection(double cx1,double cy1,double radius1, double cx2,double cy2,double radius2) /*center x,y and radius*/;
	mrpt::poses::CPose3D uwb_imu_singleA(mrpt::poses::CPose3D imu_pose, double uwb_range,double uwb_error, double imu_yaw,double imu_position_erro);
	double distance_cal(mrpt::math::TPoint2D p1,mrpt::math::TPoint2D p2);

};


#endif /* UWB_IMU_HPP */
