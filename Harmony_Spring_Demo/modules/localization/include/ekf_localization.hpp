/* +---------------------------------------------------------------------------+
  /*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   EKF_Localization.cpp
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

#ifndef EKF_LOCALIZATION_HPP
#define EKF_LOCALIZATION_HPP

class EKF_localization {
public:
  //  CMatrixDouble33 EKF_localization::G_calculation(double linear_v, double linear_w);


    CMatrixDouble31 state,state_p; // x, y, yaw , vx,vy,omega

    CMatrixDouble33 sigma; // uncertainty of state
    CMatrixDouble22 Mt;
    CMatrixDouble33 R(double linear_v, double linear_w); // process noise
    CMatrixDouble33 Q; // observation noise

    CMatrixDouble33 predictionStep(double control_v, double control_w); // x_{t+1} = g(x_t,u) and update uncertainty
    CMatrixDouble31 correctionStep(CMatrixDouble31 measurement /*wheel and IMU data*/, CMatrixDouble31 global_marker_pose /*land mark*/, CMatrixDouble33 G, CMatrixDouble33 R); // compare expected and measured values, update state and uncertainty
};

#endif /* EKF_LOCALIZATION_HPP */

