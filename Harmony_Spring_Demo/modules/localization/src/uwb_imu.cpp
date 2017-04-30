/* +---------------------------------------------------------------------------+
  /*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   uwb_imu.cpp
 * Author: Dibyendu Ghosh
 * Organization : Intel Research Lab bangalore
 * Created on 12 February, 2016, 11:50 AM
 */
// ------------------------------------------------------*/
#include <iostream>
#include <mrpt/utils.h>
#include <mrpt/math.h>
#include <mrpt/poses.h>
#include <../include/uwb_imu.hpp>
using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::utils;




mrpt::poses::CPose2D uwb_imu::uwb_imu_position(double uwb_range, double imu_yaw)
{

// Defining local variables
static double alpha = 0.0;	//radians
static double last_range;
double del_alpha;

// Assumption: constant yaw between two time steps | Anchor coord: (0,0)		
del_alpha = imu_yaw - alpha + acos((((alpha==0.0)?uwb_range:last_range)/uwb_range) * cos(alpha - imu_yaw)); //check -sign for acos

// Update values
alpha += del_alpha;
double x = uwb_range * cos(alpha);	//x pose
double y = uwb_range * sin(alpha);	//y pose
last_range = uwb_range;
mrpt::poses::CPose2D robot_pose (x,y,imu_yaw);

return robot_pose;

}
std::pair<mrpt::math::TPoint2D,mrpt::math::TPoint2D> uwb_imu::circle_intersection(double cx1,double cy1,double radius1, double cx2,double cy2,double radius2) /*center x,y and radius*/
{

	double distance=sqrt(((cx1-cx2)*(cx1-cx2))+((cy1-cy2)*(cy1-cy2))); // distance calculation between the center of Anchor (0,0) and IMU calculated position
	//std::cout << "c2cdist:  " << distance <<std::endl;

	double a=cx1,b=cy1,r0=radius1; //center of 1st circle i.e, anchor position /*radius=uwb_range*/
	double c=cx2,d=cy2,r1=radius2; //center of 2nd circle i.e, current imu position /*radius=imu_position_error*/
	mrpt::math::TPoint2D sol1,sol2;

	double root=(0.25f)*sqrt((distance+r0+r1)*(distance+r0-r1)*(distance-r0+r1)*(-distance+r0+r1));

	if(fabs(r0-r1)<=distance && distance <=fabs(r0+r1)) // circle intersect only
	{
	//http://ambrsoft.com/TrigoCalc/Circles2/Circle2.htm
		sol1.x=(a+c)/2+((c-a)*(r0*r0-r1*r1))/(2*distance*distance)+2*((b-d)/(distance*distance))*root;
		sol2.x=(a+c)/2+((c-a)*(r0*r0-r1*r1))/(2*distance*distance)-2*((b-d)/(distance*distance))*root;

		sol1.y=(b+d)/2+((d-b)*(r0*r0-r1*r1))/(2*distance*distance)+2*((a-c)/(distance*distance))*root;
		sol2.y=(b+d)/2+((d-b)*(r0*r0-r1*r1))/(2*distance*distance)-2*((a-c)/(distance*distance))*root;
	}

	std::pair<mrpt::math::TPoint2D,mrpt::math::TPoint2D>points;
	points.first=sol1;
	points.second=sol2;
	return points;
}

double uwb_imu::distance_cal(mrpt::math::TPoint2D p1,mrpt::math::TPoint2D p2)
{
	double dst=sqrt(pow((p1.x-p2.x),2)+pow((p1.y-p2.y),2));
	return dst;
}


mrpt::poses::CPose3D uwb_imu::uwb_imu_singleA(mrpt::poses::CPose3D imu_pose,double uwb_range,double uwb_error, double imu_yaw,double imu_position_erro)
{
	mrpt::math::TPoint2D anc(0,0); // anchor center

	std::pair<mrpt::math::TPoint2D,mrpt::math::TPoint2D>imuP_Uwb=uwb_imu::circle_intersection(anc.x,anc.y,uwb_range,imu_pose.x(),imu_pose.y(),imu_position_erro);
	std::pair<mrpt::math::TPoint2D,mrpt::math::TPoint2D>imuP_Uwb_erro_n=uwb_imu::circle_intersection(anc.x,anc.y,uwb_range+uwb_error,imu_pose.x(),imu_pose.y(),imu_position_erro);
    std::pair<mrpt::math::TPoint2D,mrpt::math::TPoint2D>imuP_Uwb_erro_p=uwb_imu::circle_intersection(anc.x,anc.y,uwb_range-uwb_error,imu_pose.x(),imu_pose.y(),imu_position_erro);

mrpt::poses::CPose3D robot_pose;
return robot_pose;

}
