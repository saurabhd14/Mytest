
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include "uwb_loc/uwb_hs.h"

/*
void sig_handler(int signo)
{
	if(signo == SIGINT)
		exit(0);
}
*/

int main_tmp(int argc, char **args)
{
	//signal(SIGINT, sig_handler);
	UWB uwb_pos;
	
	if(uwb_pos.initialize_pos() != 0)
		printf("ERROR: DW1000 Init Failed\n"); //TODO: handle error
		
	//TODO: calib, if required
	uwb_pos.calibrate_antenna();	

	//Vector2f uwb_position;
	//uwb_position = uwb_pos.get_pos();
	

	return 0;
}
