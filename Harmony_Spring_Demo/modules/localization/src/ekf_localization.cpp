/* +---------------------------------------------------------------------------+
  /*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   ekf_localization.cpp
 * Author: Dibyendu Ghosh
 * Organization : Intel Research Lab bangalore
 * Created on 12 February, 2016, 11:50 AM
 */
// ------------------------------------------------------*/

#include <mrpt/poses.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/system/vector_loadsave.h>
#include <mrpt/system.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/math/CQuaternion.h>
#include <iostream>
#include <math.h>
#include <mrpt/math.h>
#include "../include/ekf_localization.hpp"


using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::poses;
using namespace std;



#define dt 1 /*encoder FPS*/
#define dtx 0.10027


double control_th = 0, control_th_last = 0;

CMatrixDouble33 EKF_localization::R(double linear_v, double linear_w) {

    linear_v = linear_v + 0.00000001;
    linear_w = linear_w + 0.00000001;
    control_th += linear_w*dt;
    CMatrixDouble32 Vt;
    Vt << (-sin(control_th_last) + sin(control_th)) / linear_w, (linear_v * (sin(control_th_last) - sin(control_th))) / pow(linear_w, 2) + linear_v * cos(control_th) * dt / linear_w,
            (cos(control_th_last) - cos(control_th)) / linear_w, -(linear_v * (cos(control_th_last) - cos(control_th))) / pow(linear_w, 2) + linear_v * sin(control_th) * dt / linear_w,
            0, 1;
    CMatrixDouble33 Rt = Vt * Mt * mrpt::math::operator~(Vt);
    control_th_last = control_th;
    return Rt;
}




// odometry:
// x: distance traveled in local x-direction
// y: distance traveled in local y-direction
// phi: rotation update

double theta_p=0,theta_c=0;

CMatrixDouble33 EKF_localization::predictionStep(double control_v, double control_w) {

	control_v = control_v + 0.00000001;
	control_w = control_w + 0.00000001;
	theta_c+=control_w*dt;

	state_p(0, 0) = state(0,0)-(control_v/control_w)*sin(theta_p)+(control_v/control_w)*sin(theta_c);
	state_p(1, 0) =state(1,0)+ (control_v/control_w)*cos(theta_p)-(control_v/control_w)*cos(theta_c);
	state_p(2, 0)= mrpt::math::wrapTo2Pi(theta_c);
	state_p(2,0) = atan2(sin((double)state_p(2,0)),cos((double)state_p(2,0))); // normalize angle

    mrpt::math::CMatrixDouble33 G;
    G << 1, 0, -(control_v / control_w) * cos(theta_p)+(control_v / control_w) * cos(theta_c),
             0, 1, -(control_v / control_w) * sin(theta_p) +(control_v / control_w) * sin(theta_c),
             0, 0, 1;
    theta_p=theta_c;//state(2,0);

    return G;

}

CMatrixDouble31 EKF_localization::correctionStep(CMatrixDouble31 measurement, CMatrixDouble31 global_marker_pose, CMatrixDouble33 G, CMatrixDouble33 R) {

	 sigma = G * sigma * mrpt::math::operator~(G) + R;
    // compute expected measurement:
    CMatrixDouble31 z_exp; // z_exp = h(x)
    float psi = state_p(2, 0);
    z_exp(0, 0) =state_p(0, 0);
    z_exp(1, 0) = state_p(1, 0);
    z_exp(2, 0) = psi;


    CMatrixDouble31 err = measurement - z_exp;
    while (err(2, 0) > M_PI) err(2, 0) -= 2 * M_PI;
    while (err(2, 0) < -M_PI) err(2, 0) += 2 * M_PI;

    // dh/dx //C 
    CMatrixDouble33 H;

    H <<1, 0, 0,
    		0, 1, 0,
            0, 0, 1;

    CMatrixDouble33 K = sigma * mrpt::math::operator~(H)*(mrpt::math::operator!(H * sigma * mrpt::math::operator~(H) + Q)); // Kalman Gain

    //  correct pose estimate
    state = (state + K * (err));
    mrpt::math::CMatrixDouble33 I;
    I.setIdentity();
    sigma = (I - K * H) * sigma;
    return state;

}




