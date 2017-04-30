
#ifndef UWB_HS_H
#define UWB_HS_H

#include "../utils/utils.h"
#include "../device/dw1000.h"
#include "../application/two_way_ranging.h"
#include <mrpt/base/include/Eigen/Dense>
#include <iostream>

using namespace Eigen;

class UWB{
	public:
		//ctor
		UWB();
		void calibrate_antenna();
		int initialize_pos();
		Vector2f get_pos(/*TODO*/);
		
		//ekf
		void set_ekf_params(float _anchor_coord[6], float _x[2], float _dT); /* anchor coord | x_init | time_period */
		
		
	//private:
		//member fields
		DW1000           *dw;  
		CTwoWayRanging   *node;
		bool running;
		bool calibration_mode;
		//ekf
		float dT;
		Matrix2f P;
		Matrix2f A,W;
		//MatrixXf K;
		//MatrixXf H;
		Vector2f x;	//state
		Vector3f h_x, z;
		Vector2f a1, a2, a3;
		float Q, R;
		
		//method fields
		void PullAndReactOnStatus();
		//ekf
		void uwb_ekf(); 


};



#endif
