/* +---------------------------------------------------------------------------+
 /*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/*
 * File: main.cpp
 * Author: Dibyendu Ghosh
 * Organization : Intel Research Lab bangalore
 * Created on 12 February, 2016, 11:50 AM
 */
// ------------------------------------------------------*/

#include "device/dw1000.h"
#include "application/two_way_ranging.h"
#include <mrpt/synch/CThreadSafeVariable.h>
#include <mrpt/poses.h>
#include <mrpt/utils.h>
#include <mrpt/obs/include/mrpt/obs.h>
#include <mrpt/obs/CObservationBeaconRanges.h>
#include <mrpt/system/threads.h>
#include <mrpt/slam.h>
#include <signal.h>
#include <iostream>
#include <uwb_imu.hpp>
#include <ekf_localization.hpp>
#include <mrpt/gui/CDisplayWindowPlots.h>
/* need to be modify*/
#include "mraa.hpp"
#include <unistd.h>
#include <iostream>
#include "l298.h"
#include <signal.h>
#include "Servo.h"
#include <HcSr04.h>
#include<math.h>

#define UWB 1
#define LASER 0
#define IMU 1
#define wheel_odom 1
//#define display_map 1
//#define display_img 1
using namespace std;
using namespace mrpt;
using namespace mrpt::obs;

/* uwb device */

DW1000 *dw; // Device driver instanceSPI pins: (MOSI, MISO, SCLK, CS, IRQ)
CTwoWayRanging *node;
bool running;
void sig_handler(int signo) {
	if (signo == SIGINT)
		exit(0);
}

enum RF_STATE {
	IDLE, DISTANCE_REQUEST, DISTANCE_CALCULATE
};
enum RF_STATE rfState = DISTANCE_REQUEST;

/** Grabbing thread
 *uwb
 *imu
 *laser
 *odometry
 */

struct TThread_grabbing {
	volatile bool quit;
	volatile double Hz;
	mrpt::synch::CThreadSafeVariable<mrpt::obs::CObservationOdometryPtr> uwb_obs; //!< uwb_observation
	mrpt::synch::CThreadSafeVariable<mrpt::obs::CObservationIMUPtr> imu_obs; //!< imu_observation
};

/*  uwb grabbing thread

 */

void uwb_grabbing(TThread_grabbing &uwb_data) {
	dw = new DW1000();
	//node = new CTwoWayRanging(dw, true); //For  anchor mode
	node = new CTwoWayRanging(dw, false); //For tag mode ...\n"
	dw->setEUI(0x0102030405060708);
	if (dw->getDeviceID() != 0xDECA0130) {
		std::cout << "Quitting due invalid device id" << std::endl;
		exit(0);
	}

	if (dw->getVoltage() < 3.3) {
		std::cout << "Quitting due insufficent Voltage" << std::endl;
		exit(0);
	}

	if (node->anchor_mode) {
		node->address = 1;
		std::cout << "This node is Anchor node: " << node->address << std::endl;
	} else {
		node->address = 0;
		std::cout << "This node is a Beacon: " << node->address << std::endl;
	}
	//taging

	running = true;
	while (running && !uwb_data.quit) {
		signal(SIGINT, sig_handler);

		if (!node->anchor_mode) {
			node->RequestRanging(1);
			// node->RequestInterval(1);
		}

		uint64_t status;
		status = dw->getStatus();

		if (status & 0x4000) /* A frame was received */{
			node->CallbackRX();
			dw->writeRegister16(DW1000_SYS_STATUS, 0, 0x6F00); // clearing of receiving status bits
		}

		if (status & 0x80) /* Sending complete */{
			node->CallbackTX();
			dw->writeRegister8(DW1000_SYS_STATUS, 0, 0xF8); // clearing of sending status bits
		}

		// node->distances[0];

		mrpt::obs::CObservationOdometryPtr obsuwb =
				mrpt::obs::CObservationOdometry::Create();

		obsuwb->odometry.x() = node->distances[0]; // distance data from uwb
		uwb_data.uwb_obs.set(obsuwb);
		mrpt::system::sleep(10);

	}

}

/* end of uwb grabbing */

void imu_grabbing(TThread_grabbing &imu_data) {

	while (!imu_data.quit) {
		mrpt::obs::CObservationIMUPtr obsimu =
				mrpt::obs::CObservationIMU::Create();
		obsimu->dataIsPresent[IMU_X_ACC] = true;
		obsimu->dataIsPresent[IMU_Y_ACC] = true;
		obsimu->dataIsPresent[IMU_Z_ACC] = true;

		obsimu->dataIsPresent[IMU_YAW_VEL] = true;
		obsimu->dataIsPresent[IMU_PITCH_VEL] = true;
		obsimu->dataIsPresent[IMU_ROLL_VEL] = true;

		obsimu->dataIsPresent[IMU_YAW] = true;
		obsimu->dataIsPresent[IMU_PITCH] = true;
		obsimu->dataIsPresent[IMU_ROLL] = true;

		/*obsimu->rawMeasurements[IMU_X_ACC]=ax
		 obsimu->rawMeasurements[IMU_Y_ACC]=ay
		 obsimu->rawMeasurements[IMU_Z_ACC]=az

		 obsimu->rawMeasurements[IMU_YAW_VEL]=gx
		 obsimu->rawMeasurements[IMU_PITCH_VEL]=az
		 obsimu->rawMeasurements[IMU_ROLL_VEL]=az

		 obsimu->rawMeasurements[IMU_YAW]=yaw
		 obsimu->rawMeasurements[IMU_PITCH]=pitch
		 obsimu->rawMeasurements[IMU_ROLL]=roll

		 */

	}

}


int main() {




}
