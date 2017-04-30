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

//#include "device/dw1000.h"
//#include "application/two_way_ranging.h"
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
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/nav/planners/PlannerRRT_SE2_TPS.h>
#include <mrpt/utils/CConfigFile.h>
#include <planner_robot.hpp>
/* need to be modify*/
#include "mraa.hpp"
#include <unistd.h>
#include <iostream>
#include "l298.h"
#include <signal.h>
#include "Servo.h"
#include "HcSr04.h"
#include<math.h>
//uwb heads
#include <stdio.h>
#include <string.h>
#include "uwb_loc/uwb_hs.h"

#define UWB_SENSOR 1
#define LASER 0
#define IMU 1
#define wheel_odom 1
#define Range 200
#define tolerance 5
#define SCAN_SIZE 181
//#define display_map 1
//#define display_img 1
using namespace std;
using namespace Eigen;
using namespace mrpt;
using namespace mrpt::nav;
using namespace mrpt::utils;
using namespace mrpt::obs;


/*
 Testing code
*/
void interrupt(void * args);
void stop();

float distance_sonar[181],X[181],Y[181],x[180],y[180];
char valid_scan[181];
CObservation2DRangeScan	sonar_scan;
mrpt::maps::CSimplePointsMap map_obs;
Servo* servo = new Servo(0);
L298* l298_l = new L298(20,35,26);
L298* l298_r = new L298(14,25,13);
HCSR04* sonar = new HCSR04(33,47, &interrupt);

mrpt::poses::CPose3D r_pose = mrpt::poses::CPose3D(0.0,0.0,0.0,0,0,0);




/* uwb device */

void sig_handler(int signo) {
	if (signo == SIGINT){
		sonar->m_doWork = 1;
		stop();
		exit(0);
	}
}


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




void Motion(int L, int R)
{
	l298_l-> setSpeed(L);
	l298_l-> setDirection(L298::DIR_FW);
	l298_l-> enable(true);
	l298_r->setSpeed(R);
	l298_r-> setDirection(L298::DIR_FW);
	l298_r-> enable(true);
	
}
void stop(){
	l298_l-> setDirection(L298::DIR_NONE);
	l298_r-> setDirection(L298::DIR_NONE);
	
}
void interrupt (void * args)
{
	sonar-> ackEdgeDetected();
}

mrpt::maps::CSimplePointsMap generate_map(){
	mrpt::maps::CSimplePointsMap map_obs;
		  int count = 0;
	     servo->setAngle (0);
	     //sleep(2);

	     for(int i=0;i<=180;i+=1){
	     	servo->setAngle(i);
	     	usleep(15000);
	         distance_sonar[i] = sonar->getDistance(CM)/100;
	         usleep(100000);
	         std::cout << "Distance Sonar "<< distance_sonar[i] << " angle "<< i << std::endl;
	     }
	     sleep(1);
	     servo->setAngle(0);
	     sleep(1);


	     //delete sonar;
	     //delete servo;

	     for (int i = 0; i<=180; i++)
	     {
	     	if(distance_sonar[i]>4)
	     	{
	     		valid_scan[i] = '0';
	     		distance_sonar[i] = 0;
	     	}
	     	else
	     		valid_scan[i] = '1';
	     }
	     for(int i=1;i<180;i+=1){
	     	if(valid_scan[i] != '0' && (distance_sonar[i+1] != 0 || distance_sonar[i-1] != 0 )){
	         	if(distance_sonar[i] - distance_sonar[i+1] > abs(10) && distance_sonar[i] - distance_sonar[i-1] > abs(10) )
	         	distance_sonar[i] = (distance_sonar[i+1] + distance_sonar[i-1])/2;
	     		}
	         }
	    /* for(int i=0;i<=180;i++){
	     	    	 std::cout << distance_sonar[i] <<","<< std::endl;
	     	     }*/
	     printf("After 1st correction\n");
	     /*for(int i=0;i<176;i+=1)
	     {
	    	 double temp = distance_sonar[i]+distance_sonar[i+1]+distance_sonar[i+2]+distance_sonar[i+3]+distance_sonar[i+4];
	    	 temp = temp/5;
	    	 distance_sonar[i] = temp;
	     }*/
	     for(int i=0; i<=180;i++)
	     {
	    	 if(i<90)
	    	 {
	    		X[i] = distance_sonar[i]*cos(i*(3.14/180));
	    		Y[i] = distance_sonar[i]*sin(i*(3.14/180));
	    	 }

	    	 else
	    	 {
	    		 X[i] = - distance_sonar[i]*cos((180-i)*(3.14/180));
	    		 Y[i] = distance_sonar[i]*sin((180-i)*(3.14/180));
	    	 }
	     }
	     /*for(int i=0;i<=180;i++)
	     {
	    	 printf("X = %f , Y = %f\n",X[i],Y[i]);
	     }*/

	     printf("converted to X Y\n");

	     /*for(int x =0;x<=800;x=x+5)
	     {
	    	 for(int y=0;y<=400;y=y+5)
	    	 {
	    		 for(int i=0;i<=180;i++)
	    		 {
	    			 if(fabs(X[i]-(x-400)) < 5 && fabs(Y[i]-y) < 5)
	    				 count++;
	    		 }
	    		 //printf("count: %d, X: ,Y: \n",count,x,y);
	    		 if(count>3)
	    			 obs[x][y] = 1;
	    		 else
	    			 obs[x][y] = 0;

	    		 count = 0;
	    	 }
	     }
*/

	     for(int i=0;i<=180;i++)
	     {
	    	 for(int j=0;j<=180;j++)
	    	 {
	    		 if(fabs(X[i]-X[j]) < 0.05 && fabs(Y[i]-Y[j]) < 0.05)
	    		 {
	    			 count++;
	    		 }
	    	 }
	    	 if(count<6){
	    		 x[i] = 0;
	    		 y[i] = 0;
	    		 valid_scan[i] = '0' ;
	    	 }
	    	 else{
	    		 x[i] = X[i];
	    		 y[i] = Y[i];
	    		 valid_scan[i] = '1';
	    	 }
	    	 count = 0;
	     }


	     /*printf("Grid obstacle map\n");
	     for(int i=0;i<=180;i++)
	     	     {
	     	    	 printf("%f,\n",x[i]);
	     	     }

	     printf("\n\n\n");

	     for(int i=0;i<=180;i++)
	     	     {
	     	   	  	 printf("%f,\n",y[i]);
	     	     }*/
	     printf("converting to Polar\n\n");
	     
	     for(int i=0;i<=180;i++){
	    	 distance_sonar[i] = sqrt((x[i]*x[i])+(y[i]*y[i]));
	    	 if(distance_sonar[i] > 0.0)
	    		 valid_scan[i] = '1';
	    	 else
	    		 valid_scan[i] = '0';
	     }

	     printf("\n\n");

	     for(int i=0;i<=180;i++){
				std::cout << distance_sonar[i] <<","<< std::endl;
	     	    std::cout << valid_scan[i] <<","<< std::endl;
	     	   /*if(valid_scan[i] != '0')
	     		   printf("1 ,\n");
	     	   else
	     		  printf("0 ,\n");*/
	     	     }
	     /*for(int i=0;i<=180;i+=1){
	     	std::cout << distance_sonar[i] <<","<< std::endl;
	             }*/
	     /*for(int y=400;y>=400;y--)
	     {
	    	 for(int x=-400;x<=400;x++) {
	    		 if(obs[x][y]==1)
	    			 printf("1");
	    		 else
	    			 printf("0");
	   	   	 }
	     }*/
	   
	std::cout<<" Scan Complete" <<std::endl;
	
	sonar_scan.aperture = M_PIf;
	sonar_scan.rightToLeft = true;
	sonar_scan.validRange.resize( SCAN_SIZE );
	sonar_scan.scan.resize(SCAN_SIZE);
	
	//std::cout<<" ASSERT Start "<<std::endl;
	ASSERT_( sizeof(distance_sonar) == sizeof(float) * SCAN_SIZE );
	//ASSERT_( sizeof(valid_scan) == sizeof(char) * SCAN_SIZE );

	//std::cout<<" ASSERT complete "<<std::endl;

	memcpy( &sonar_scan.scan[0], distance_sonar, sizeof(distance_sonar) );
	memcpy( &sonar_scan.validRange[0], valid_scan, sizeof(valid_scan) );
	map_obs.insertObservation(&sonar_scan);
	map_obs.save2D_to_text_file("test.txt");
	return map_obs;
		
}


int main() {
	
Planner_robot test_p;
int counter = 0;

//uwb init

UWB uwb_pos;
Vector2f uwb_position;
printf("ABHIROOP: config uwb driver start\n");
if(uwb_pos.initialize_pos() != 0)
	printf("ERROR: DW1000 Init Failed\n"); //TODO: handle error
//TODO: antenna calib, if required
//uwb init		
printf("ABHIROOP: config uwb driver done\n");


mrpt::poses::CPose2D current_pose = mrpt::poses::CPose2D(0.0,0.0,0);
mrpt::poses::CPose2D final_pose = mrpt::poses::CPose2D(-1.0,0.3,0);

string config_robot= "../config/config.ini";
printf("ABHIROOP: planner config start\n");
// Parameters:
test_p.planner_m.loadConfig( mrpt::utils::CConfigFile(config_robot));
printf("ABHIROOP: planner config done\n");
test_p.planner_m.params.maxLength = 2.0;
test_p.planner_m.params.minDistanceBetweenNewNodes = 0.50;
test_p.planner_m.params.minAngBetweenNewNodes = mrpt::utils::DEG2RAD(20);
test_p.planner_m.params.goalBias = 0.85;

//// End criteria:
test_p.planner_m.end_criteria.acceptedDistToTarget = 0.25;
test_p.planner_m.end_criteria.acceptedAngToTarget  = DEG2RAD(180); // 180d=Any orientation is ok
test_p.planner_m.end_criteria.maxComputationTime = 15.0;
test_p.planner_m.end_criteria.minComputationTime = 0.0; // 0=accept first found acceptable solution
printf("ABHIROOP: planner init start\n");
//// Init planner:
//// -----------------------------
test_p.planner_m.initialize();
//printf("ABHIROOP: planner init done\n");
mrpt::poses::CPose2D vw;
double vl = 0.0, vr = 0.0;
bool kuchbhi = false;
mrpt::maps::CSimplePointsMap simplemap;
    while(1)
    {
		if(!kuchbhi /*counter%10 == 0*/)
		
		{	
			printf("ABHIROOP: generating map\n");
			simplemap.clear();
			simplemap=generate_map();
			simplemap.save2D_to_text_file("test1.txt");
			kuchbhi = true;
		}
		

		 printf("ABHIROOP: collecting uwb coordinates\n");
                uwb_position = uwb_pos.get_pos();
                cout<<"X pos: "<<uwb_position(0)<<"\tY pos: "<<uwb_position(1)<<endl;
                current_pose = mrpt::poses::CPose2D(uwb_position(0),uwb_position(1),0);


		std::cout<<" Size " << simplemap.size()<<std::endl;
		printf("ABHIROOP: path planning...\n");
		vw=test_p.Path_Planner(current_pose,final_pose,simplemap);
		std::cout << " v: " << vw.x()  << " w: " << vw.phi() <<std::endl;

		double linear_velocity = vw.x();
		double angular_velocity = vw.phi();

		if(fabs(angular_velocity) < 0.1)
		{
			vl = linear_velocity - 0.1*angular_velocity;
			vr = linear_velocity + 0.1*angular_velocity;
		}
		else
		{
			vl = 2*linear_velocity ;
			vr = 2*linear_velocity ;
		}
		int ml = vl*100/0.15;
		int mr = vr*100/0.15;
		std::cout << " vl: " << ml  << " vr: " << mr <<std::endl;

		printf("ABHIROOP: carrying out motion commands\n");
		Motion(ml,mr);

		//sleep(2);
		//uwb x/y grab
		/*printf("ABHIROOP: collecting uwb coordinates\n");
		uwb_position = uwb_pos.get_pos();
		cout<<"X pos: "<<uwb_position(0)<<"\tY pos: "<<uwb_position(1)<<endl;
		current_pose = mrpt::poses::CPose2D(uwb_position(0),uwb_position(1),0);
		*/
		counter++;
		//stop();
		/*if (current_pose.x()<-1.0)
			kuchbhi = false;*/
    }
   //while(1)
   //{

		//sleep(2); 
		//Vector2f uwb_position = uwb_pos.get_pos();
		//std::cout<<"X pos: "<<uwb_position(0)<<" Y pos: "<<uwb_position(1)<<std::endl;
		//printf ("\n");
		////if(uwb_position(0) < 0)
		////Motion(100,100);
		////else
		////{
			////if(uwb_position(1)>0.6)
			////Motion(15,80);
			//////else
			//////Motion(100,100);
		////}
	////}
	
    ////Motion(100,100);
	//}
}
