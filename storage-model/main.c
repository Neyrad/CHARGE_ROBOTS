//The C main file for a ROSS model
//This file includes:
// - definition of the LP types
// - command line argument setup
// - a main function


#include "ross.h"
#include "model.h"




const char* path_to_log_folder  = "/mnt/c/Dev/Base/pygame/Simulation_History";
const char* path_to_room_file   = "field.csv";
const char* path_to_robots_file = "robots.csv";
const char* path_to_pairs       = "log.csv";





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

unsigned int setting_1 = 0;

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
	ParsePairs(path_to_pairs);;
	
    RobotsInit();
    assert(Robots.N);

    PrintMap(path_to_log_folder);
    RobotsPrint();

    int i;
    int num_lps_per_pe;

    tw_opt_add(model_opts);
    tw_init(&argc, &argv);

    num_lps_per_pe = Robots.N + 1; //n robots + command center

    tw_define_lps(num_lps_per_pe, sizeof(message));

    g_tw_lp_typemap = &model_typemap;

    for (int i = 0; i < g_tw_nlp; ++i)
        tw_lp_settype(i, &model_lps[0]);

    tw_run();
    tw_end();

    PrintNSteps(path_to_log_folder);
	printf("Final global time is %d\n", glb_time);
}
