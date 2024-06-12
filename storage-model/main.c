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
	LeadAcid
	LiCoO2
*/


// TODO: CELL_IN, CELL_OUT, CELL_CHARGER should be ALLOWED to cross in routing, BUT should have a higher G cost than CELL_EMPTY and lower than ANTI-DEADLOCK
// so GetGScore(robot, CELL_OUT) returns BIG_NUMBER / 2 or something.
// In such case avoiding CELL_IN, CELL_OUT, CELL_CHARGER while routing would be preffered, but crossing them is still better than a dead-lock.



int main(int argc, char* argv[])
{
	InitROSS();
    SimulateROSS(argc, argv);
	FinalizeROSS();
}
