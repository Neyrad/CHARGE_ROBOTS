#include "model.h"

//The C extras file for a ROSS model
//This file includes:
// - all functions that are not from ROSS

extern int glb_time;

void FreePairs()
{	
	for (int i = 0; i < MAX_INPUT_LENGTH; ++i)
		free(pairs.elem[i]);
	free(pairs.elem);
}

void SendMessageContents(tw_lpid receiver, tw_lp* lp, double ts, lp_type type, double contents)
{
    tw_event* Event   = tw_event_new(receiver, ts, lp);
    message* Message  = tw_event_data(Event);
    Message->type     = type;
    Message->contents = contents;
    Message->sender   = lp->gid;
    tw_event_send(Event);
}

void SendMessage(tw_lpid receiver, tw_lp* lp, double ts, lp_type type)
{
    SendMessageContents(receiver, lp, ts, type, 0);
}

void SimulateROSS(int argc, char* argv[])
{
	ValidateMacros();
	int i, num_lps_per_pe;
    tw_opt_add(model_opts);
    tw_init(&argc, &argv);
    num_lps_per_pe = Robots.N + 1; //n robots + command center
    tw_define_lps(num_lps_per_pe, sizeof(message));
    g_tw_lp_typemap = &model_typemap;
    for (int i = 0; i < g_tw_nlp; ++i)
        tw_lp_settype(i, &model_lps[0]);
    tw_run();
    tw_end();
	printf("\nFinal global time is %d days\n", glb_time / (9000 * 24));
}

void Free()
{
	FreePairs();
}

void FinalizeROSS()
{
	//PrintNBoxesDelivered();
	Free();
	fclose(LogFile);
}

/*
void BatteryDeath(struct _robot* robot)
{
	warehouse.robots[robot->y][robot->x] = CELL_EMPTY;
	robot->battery.dead = true;
}


void PrintNBoxesDelivered()
{
	FILE* f = fopen("/mnt/c/Dev/Base/graph/boxes_delivered.csv", "a");
	fprintf(f, "%d,", glb_time / 9000);
	for (int i = 0; i < Robots.N - 1; ++i)
		fprintf(f, "%d,", Robots.elem[i].boxes_delivered);
	fprintf(f, "%d\n", Robots.elem[Robots.N - 1].boxes_delivered);
	fclose(f);
}
*/
