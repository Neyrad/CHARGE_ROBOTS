//The C main file for a ROSS model
//This file includes:
// - definition of the LP types
// - command line argument setup
// - a main function

//includes
#include "ross.h"
#include "model.h"




const char* path_to_log_folder  = "/mnt/c/Dev/Base/pygame/Simulation_History";
const char* path_to_room_file   = "field.csv";
const char* path_to_robots_file = "robots.csv";
const char* path_to_pairs       = "log.csv";




// Define LP types
//   these are the functions called by ROSS for each LP
//   multiple sets can be defined (for multiple LP types)
tw_lptype model_lps[] = {
  {
    (init_f) model_init,
    (pre_run_f) NULL,
    (event_f) model_event,
    (revent_f) model_event_reverse,
    (commit_f) NULL,
    (final_f) model_final,
    (map_f) model_map,
    sizeof(state)
  },
  { 0 },
};

//Define command line arguments default values
unsigned int setting_1 = 0;

//add your command line opts
const tw_optdef model_opts[] = {
    TWOPT_GROUP("ROSS Model"),
    TWOPT_UINT("setting_1", setting_1, "first setting for this model"),
    TWOPT_END(),
};


extern int glb_time;


int main (int argc, char* argv[])
{
	srand(time(NULL));
	
    Parse(path_to_room_file, storage.room);
	Parse(path_to_robots_file, storage.robots);
	
	ParsePairs(path_to_pairs);
	PrintPairs();
	
    RobotsInit();
    assert(Robots.N);

    PrintMap(path_to_log_folder);
    RobotsPrint();

    int i;
    int num_lps_per_pe;

    tw_opt_add(model_opts);
    tw_init(&argc, &argv);

    //assume 1 lp per node
    num_lps_per_pe = Robots.N + 1; //n robots + command center

    //set up LPs within ROSS
    tw_define_lps(num_lps_per_pe, sizeof(message));
    // note that g_tw_nlp gets set here by tw_define_lps

    // IF there are multiple LP types
    //    you should define the mapping of GID -> lptype index
    g_tw_lp_typemap = &model_typemap;

    // set the global variable and initialize each LP's type
    //  g_tw_lp_types = model_lps;
    //  tw_lp_setup_types();

    //printf("g_tw_nlp == %ld\n", g_tw_nlp);
    for (int i = 0; i < g_tw_nlp; ++i)
        tw_lp_settype(i, &model_lps[0]);

    // Do some file I/O here? on a per-node (not per-LP) basis
    tw_run();
    tw_end();

    PrintNSteps(path_to_log_folder);
	printf("Final global time is %d\n", glb_time);
}
