//The C main file for a ROSS model
//This file includes:
// - a main function

#include "ross.h"
#include "model.h"

//---------------------------Path Configurations---------------------------//

const char* path_to_log_folder  = "/mnt/c/Dev/Base/pygame/Simulation_History";
const char* path_to_room_file   = "field.csv";
const char* path_to_robots_file = "robots.csv";
const char* path_to_pairs       = "log.csv";

/*
	Available Battery Types:

	LiFePO4
	LiNiMnCoO2
*/


int main(int argc, char* argv[])
{
	InitROSS();
    SimulateROSS(argc, argv);
	FinalizeROSS();
}
