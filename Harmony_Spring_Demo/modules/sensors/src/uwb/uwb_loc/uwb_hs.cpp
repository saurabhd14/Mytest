// by Matthias Grob & Manuel Stalder - ETH Zürich - 2015
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "uwb_hs.h"

using namespace Eigen;

//----------------------------------------------------------


UWB::UWB()
{
			calibration_mode = false;
			//TODO: remove this: h_x(3), z(3), a1(2), a2(2), a3(2);
			a1(0) = 0.0f;
			a1(1) = 1.13f;
			a2(0) = 2.23f;
			a2(1) = 0.0f; 
			a3(0) = -2.23f;
			a3(1) = 0.0f;
			P << 1,0,
				 0,1;
			x << 1.3f,
			     0.3f;
			A.setIdentity();
			dT = 2; //TODO: dynamic dt update??
			W = dT * Matrix2f::Identity();
			//H(3,2);
			//H.setZero();
			Q = 0.1;
			R = 0.0001;
			z.setZero();
			//K(2,3);

}


void UWB::set_ekf_params(float _anchor_coord[6], float _x[2], float _dT)
{
	a1(0) = _anchor_coord[0];
	a1(1) = _anchor_coord[1];
	a2(0) = _anchor_coord[2];
	a2(1) = _anchor_coord[3];
	a3(0) = _anchor_coord[4];
	a3(1) = _anchor_coord[5];
	x(0) = _x[0];
	x(1) = _x[1];
	dT = _dT;

}

void UWB::uwb_ekf()
{
		//TODO: sort out this declaration in Eigen
		static MatrixXf K(2,3);
		static MatrixXf H(3,2);

	//load new distances TODO: *********filter bad range reading spikes!*********
	for(int k=0; k<MAX_CONNECTIONS; k++)
		z(k) = node->distances[k+1];
//printf("ABHIROOP: EKF begins\n");
	x = A * x;
	P = ( A * P * A.transpose() ) + ( W * Q * W.transpose() );
	
	float euclid1 = sqrt( (x(0)-a1(0))*(x(0)-a1(0)) + (x(1)-a1(1))*(x(1)-a1(1)) );
	float euclid2 = sqrt( (x(0)-a2(0))*(x(0)-a2(0)) + (x(1)-a2(1))*(x(1)-a2(1)) );
	float euclid3 = sqrt( (x(0)-a3(0))*(x(0)-a3(0)) + (x(1)-a3(1))*(x(1)-a3(1)) );
	
	H << (x(0)-a1(0))/euclid1 , (x(1)-a1(1))/euclid1,
		 (x(0)-a2(0))/euclid2 , (x(1)-a2(1))/euclid2,
		 (x(0)-a3(0))/euclid3 , (x(1)-a3(1))/euclid3;
	
	h_x << euclid1,
		   euclid2,
		   euclid3;
	
	K = P * H.transpose() * ( (H * P * H.transpose()) + (R * Matrix3f::Identity()) ).inverse();
	x = x + K * (z - h_x);
	P = (Matrix2f::Identity() - K * H) * P;
//printf("ABHIROOP: EKF ends\n");
}


//----------------------------------------------------------




void UWB::PullAndReactOnStatus()
{
	uint64_t status;

	status = dw->GetStatus();

	if(status & 0x4000) /* A frame was received */
	{
		node->CallbackRX();
		dw->Write16BitToRegister(DW1000_SYS_STATUS, 0, 0x6F00);              // clearing of receiving status bits
	}

	if(status & 0x80) /* Sending complete */
	{
		node->CallbackTX();
		dw->Write8BitToRegister(DW1000_SYS_STATUS, 0, 0xF8);                 // clearing of sending status bits
	}
}




int UWB::initialize_pos()
{


	/* ************* Setup the dw1000 module ************* */
	SRadioConfig *dw1000_configuration = new SRadioConfig();
	dw1000_configuration->pan_id = SYSTEM_WIDE_PAN_ID;
	dw1000_configuration->energy_scan_mode = false;
	dw1000_configuration->smart_tx_power_enable = false;
	dw1000_configuration->channel = 5;
	dw1000_configuration->low_pulse_repetition_frequency = true;
	dw1000_configuration->sfd_style = 0; //dw
	dw1000_configuration->data_rate = 2; //6.8mbps
	dw1000_configuration->preamble_length = 256; //short preamble for high data rate
	dw1000_configuration->max_frame_length = 127;

	dw1000_configuration->sfd_detection_timeout = dw1000_configuration->preamble_length+64+1; //4096 + 64 + 1;
	dw1000_configuration->preamble_detection_timeout = 0;
	dw1000_configuration->receiver_auto_reenable = true;

	dw1000_configuration->lde_noise_peak_multiplier = 3;
	dw1000_configuration->lde_noise_level_multiplier = 13;

	dw1000_configuration->antenna_delay_receiver = -0.02648498;//0.516260/2;
	dw1000_configuration->antenna_delay_transmitter = -0.02648498;//0.516260/2;
	dw1000_configuration->enable_event_counter_diagnostics = true;
	dw1000_configuration->enable_leds = true;

	/* Frame filtering options TODO: Beacon frame is not available yet */
	dw1000_configuration->enable_frame_filtering = true;

	printf("DEBUG: Running in tag mode ...\n");
	/* Replace here with your address distribution system */
	dw1000_configuration->address = 0;
	dw = new DW1000(dw1000_configuration);
	node = new CTwoWayRanging(dw, false);
	
	printf("DEBUG: Device information: \n");
	printf("DEBUG: DEVICE_ID: 0x%X\r\n", dw->GetDeviceID());
	dw->SetEUI(0x0102030405060708);
	printf("DEBUG: EUI: %016llX\r\n", dw->GetEUI());
	printf("DEBUG: Voltage: %fV\r\n", dw->GetVoltage());

	if(dw->GetDeviceID() != 0xDECA0130)
	{
		printf("Quitting due invalid device id!\n");
		return -1;
	}

	if(dw->GetVoltage() < 3.3)
	{
		printf("Quitting due insufficient Voltage!\n");
		return -2;
	}

	
	//TODO: signal(SIGINT, sig_handler);
	
	
	running = true;
	return 0;
}


void UWB::calibrate_antenna()
{

	/* Calibration mode relevant variables */
	double calibration_distance;
	
	printf("INFO: Running in calibration mode\n");
	printf("INFO: Please make sure to take existing values set in your configuration into account!\n");

	dw->config->antenna_delay_receiver = 0.0;
	dw->config->antenna_delay_transmitter = 0.0;
	
	/* Relevant if running in calibration mode */
	for(int j = 1 ; j<=MAX_CONNECTIONS ; j++)
	{
		calibration_distance = dw->GiveCalibrationDistance();
		printf("Running in calibration mode...\n\n Please set the distance between the calibrating devices to %4f meter and press enter...\n", calibration_distance);
		getchar();
		printf("Starting measurement for anchor %d...\n",j);
		while(1)
		{
			if(node->RequestCalibrationValue(j, calibration_distance))
				break;
		}
		node->ranging_count = 1;
		node->sum_ranges = 0;
	}
	calibration_mode = false;
}





Vector2f UWB::get_pos(/*TODO*/)
{

	while(running)
	{

		//node->RequestRanging(1);
		if(node->requestRangingAll())
        {
			//TODO: Comment this part out after debug
           // for(int i=1;i<=MAX_CONNECTIONS;i++)
			//	printf("Anchor %d: %f\n",i,node->distances[i]);
				
			uwb_ekf();
			return x;	
				
		}

		PullAndReactOnStatus(); /* Pulling the status register manually */
	}
	
	Vector2f tv;
	return tv.setZero(); //TODO: other returns

}


