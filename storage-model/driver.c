//The C driver file for a ROSS model
//This file includes:
// - an initialization function for each LP type
// - a forward event function for each LP type
// - a reverse event function for each LP type
// - a finalization function for each LP type

//Includes
#include <stdio.h>

#include "ross.h"
#include "model.h"

//Helper Functions
void SWAP(double* a, double* b)
{
    double tmp = *a;
    *a = *b;
    *b = tmp;
}

int dest = 0;
extern const char* path_to_log_folder;

int glb_time = 0;
int RobotResponded[MAX_ROBOTS];

//Init function
// - called once for each LP
// ! LP can only send messages to itself during init !
void model_init (state *s, tw_lp *lp)
{
    if (lp->gid == 0)
    {
        s->type = COMMAND_CENTER;
		
		for (int i = 0; i < Robots.N; ++i)
			RobotResponded[i] = FALSE;
		
        printf("COMMAND_CENTER is initialized\n");
    }
    else
    {
        s->type = ROBOT;
        assert(lp->gid <= Robots.N);
		
		Robots.data[lp->gid - 1].state      = STOP;
		Robots.data[lp->gid - 1].battery    = BATTERY_CAPACITY;
		Robots.data[lp->gid - 1].capacity   = BATTERY_CAPACITY;
		Robots.data[lp->gid - 1].charging   = FALSE;
		AssignDest(&Robots.data[lp->gid - 1], CELL_BOX);
    
        printf("ROBOT #%ld is initialized\n", lp->gid);
    }

    int self = lp->gid;

    // init state data
    s->value = -1;

    s->got_msgs_ROTATE	 = 0;
    s->got_msgs_MOVE_U   = 0;
	s->got_msgs_MOVE_D   = 0;
	s->got_msgs_MOVE_L   = 0;
	s->got_msgs_MOVE_R   = 0;
    s->got_msgs_BOX_GRAB = 0;
    s->got_msgs_BOX_DROP = 0;
    s->got_msgs_RECEIVED = 0;
    s->got_msgs_INIT     = 0;
	s->got_msgs_NOP      = 0;

    if (lp->gid == 0)
		for (int i = 1; i <= Robots.N; ++i)
			SendMessage(i, lp, glb_time, INIT);
}

//Forward event handler
void model_event (state* s, tw_bf* bf, message* in_msg, tw_lp* lp)
{
    int self = lp->gid;
    bool is_executed = FALSE;
    // initialize the bit field
    *(int*)bf = (int)0;

    // update the current state
    // however, save the old value in the 'reverse' message
    SWAP(&(s->value), &(in_msg->contents));

    // handle the message
    switch(s->type)
    {
        case COMMAND_CENTER:
            switch (in_msg->type) //msg counter
            {
                case RECEIVED:
                    ++s->got_msgs_RECEIVED; 
                    break;
                case INIT:
                    ++s->got_msgs_INIT; 
                    break;
                default:
                    printf("COMMAND CENTER: Unhandled forward message type %d\n", in_msg->type);
            }

            if (pairs.eof)
				return;

			RobotResponded[in_msg->sender-1] = TRUE;
			if (EveryoneResponded(RobotResponded, Robots.N))
			{
				PrintMap(path_to_log_folder);
				glb_time += 1;
				
				for (int i = 0; i < Robots.N; ++i)
					RobotResponded[i] = FALSE;
				
				
				for (int i = 1; i <= Robots.N; ++i)
				{
					message_type cmd = CalcNextMove2(&Robots.data[i-1]);
					SendMessage(i, lp, glb_time, cmd);
				}
			}
            break;

        case ROBOT:
            {
            struct _robot* This = &Robots.data[self-1];
            printf("Robot #%d: battery level is %d/%d\n", self, This->battery, This->capacity);
			switch (in_msg->type)
            {
                case ROTATE:
                    ++s->got_msgs_ROTATE;
					if (This->state == MOTION)
						This->battery -= STOP_MOTION_COST;
					This->state = STOP;
					
					if (This->orientation == VER)
					{
						This->orientation = HOR;
						storage.robots[This->y][This->x] = This->carries_box ? CELL_ROBOT_WITH_BOX_HOR : CELL_ROBOT_HOR;
					}
					else
					{
						This->orientation = VER;
						storage.robots[This->y][This->x] = This->carries_box ? CELL_ROBOT_WITH_BOX_VER : CELL_ROBOT_VER;
					}
					
					This->battery -= ROTATE_COST;
                    break;
                case MOVE_U:
                    ++s->got_msgs_MOVE_U;
					assert(This->orientation == VER);
					assert(This->y - 1 >= 0);
                    switch(storage.room[This->y - 1][This->x])
                    {
                        case CELL_EMPTY:
						case CELL_BOX:
						case CELL_CONTAINER:
						case CELL_CHARGER:
							if (storage.robots[This->y - 1][This->x] == CELL_EMPTY)
							{
								storage.robots[This->y - 1][This->x] = This->carries_box ? CELL_ROBOT_WITH_BOX_VER : CELL_ROBOT_VER;
								storage.robots[This->y    ][This->x] = CELL_EMPTY;
								This->y = This->y - 1;
								
								if 		(This->state == MOTION)
									This->battery -= KEEP_MOTION_COST;
								else if (This->state == STOP)
									This->battery -= START_MOTION_COST;
								
								This->state = MOTION;
							}
                            break;
                        case CELL_WALL:
							This->state = STOP;
							break;
                        default:
                            break;
                    }
                    break;
				case MOVE_D:
                    ++s->got_msgs_MOVE_D;
					assert(This->orientation == VER);
					assert(This->y + 1 < storage.height);
                    switch(storage.room[This->y + 1][This->x])
                    {
                        case CELL_EMPTY:
						case CELL_BOX:
						case CELL_CONTAINER:
						case CELL_CHARGER:
							if (storage.robots[This->y + 1][This->x] == CELL_EMPTY)
							{
								storage.robots[This->y + 1][This->x] = This->carries_box ? CELL_ROBOT_WITH_BOX_VER : CELL_ROBOT_VER;
								storage.robots[This->y    ][This->x] = CELL_EMPTY;
								This->y = This->y + 1;
								
								if 		(This->state == MOTION)
									This->battery -= KEEP_MOTION_COST;
								else if (This->state == STOP)
									This->battery -= START_MOTION_COST;
								
								This->state = MOTION;
							}
                            break;
                        case CELL_WALL:
							This->state = STOP;
							break;
                        default:
                            break;
                    }
                    break;
				case MOVE_L:
                    ++s->got_msgs_MOVE_L;
					assert(This->orientation == HOR);
					assert(This->x - 1 >= 0);
                    switch(storage.room[This->y][This->x - 1])
                    {
                        case CELL_EMPTY:
						case CELL_BOX:
						case CELL_CONTAINER:
						case CELL_CHARGER:
							if (storage.robots[This->y][This->x - 1] == CELL_EMPTY)
							{
								storage.robots[This->y][This->x - 1] = This->carries_box ? CELL_ROBOT_WITH_BOX_HOR : CELL_ROBOT_HOR;
								storage.robots[This->y][This->x    ] = CELL_EMPTY;
								This->x = This->x - 1;
								
								if 		(This->state == MOTION)
									This->battery -= KEEP_MOTION_COST;
								else if (This->state == STOP)
									This->battery -= START_MOTION_COST;
								
								This->state = MOTION;
							}
                            break;
                        case CELL_WALL:
							This->state = STOP;
							break;
                        default:
                            break;
                    }
                    break;
				case MOVE_R:
                    ++s->got_msgs_MOVE_R;
					assert(This->orientation == HOR);
					assert(This->x + 1 < storage.length);
                    switch(storage.room[This->y][This->x + 1])
                    {
                        case CELL_EMPTY:
						case CELL_BOX:
						case CELL_CONTAINER:
						case CELL_CHARGER:
							if (storage.robots[This->y][This->x + 1] == CELL_EMPTY)
							{
								storage.robots[This->y][This->x + 1] = This->carries_box ? CELL_ROBOT_WITH_BOX_HOR : CELL_ROBOT_HOR;
								storage.robots[This->y][This->x]     = CELL_EMPTY;
								This->x = This->x + 1;
								
								if 		(This->state == MOTION)
									This->battery -= KEEP_MOTION_COST;
								else if (This->state == STOP)
									This->battery -= START_MOTION_COST;
								
								This->state = MOTION;
							}
                            break;
                        case CELL_WALL:
							This->state = STOP;
							break;
                        default:
                            break;
                    }
                    break;
                case BOX_GRAB:
                    ++s->got_msgs_BOX_GRAB;
					if (This->state == MOTION)
						This->battery -= STOP_MOTION_COST;
					This->state = STOP;
					
                    if (storage.room[This->y][This->x] == CELL_BOX && This->carries_box == FALSE)
                    {
                        This->carries_box = TRUE;
						if (This->orientation == VER)
							storage.robots[This->y][This->x] = CELL_ROBOT_WITH_BOX_VER;
						else
							storage.robots[This->y][This->x] = CELL_ROBOT_WITH_BOX_HOR;
                        printf("ROBOT #%d: grab the box.\n", self);
						AssignDest(&Robots.data[self-1], CELL_CONTAINER);
                    }
                    break;
                case BOX_DROP:
                    ++s->got_msgs_BOX_DROP;
					if (This->state == MOTION)
						This->battery -= STOP_MOTION_COST;
					This->state = STOP;
					
                    if (storage.room[This->y][This->x] == CELL_CONTAINER && This->carries_box == TRUE)
                    {
                        This->carries_box = FALSE;
						if (This->orientation == VER)
							storage.robots[This->y][This->x] = CELL_ROBOT_VER;
						else
							storage.robots[This->y][This->x] = CELL_ROBOT_HOR;
                        
						printf("ROBOT #%d: drop the box.\n", self);
						
						if (Robots.data[self-1].battery < TIME_TO_CHARGE_THRESHOLD)
							AssignDest(&Robots.data[self-1], CELL_CHARGER);
						else
							AssignDest(&Robots.data[self-1], CELL_BOX);
                    }
                    break;
                case INIT:
                    ++s->got_msgs_INIT;
					if (This->state == MOTION)
						This->battery -= STOP_MOTION_COST;
					This->state = STOP;
                    break;
				case NOP:
					++s->got_msgs_NOP;
					if (This->state == MOTION)
						This->battery -= STOP_MOTION_COST;
					This->state = STOP;
					break;
                default:
                    printf("ROBOT #%d: Unhandled forward message of type %d\n", self, in_msg->type);
            }

            if (in_msg->sender == 0) //the message came from the command center
                SendMessage(0, lp, glb_time, RECEIVED);

            break;
        }
    } 
}

//Reverse Event Handler
void model_event_reverse (state *s, tw_bf *bf, message *in_msg, tw_lp *lp) {
/*  int self = lp->gid;

  // undo the state update using the value stored in the 'reverse' message
  SWAP(&(s->value), &(in_msg->contents));

  // handle the message
 

  // don't forget to undo all rng calls
  tw_rand_reverse_unif(lp->rng);  */
}

//report any final statistics for this LP
void model_final(state* s, tw_lp* lp)
{
    int self = lp->gid;
    if      (s->type == COMMAND_CENTER)
        printf("COMMAND_CENTER: got %d messages of type ROTATE\n",        s->got_msgs_ROTATE);
	else if (s->type == ROBOT)
		printf("ROBOT #%2d:      got %d messages of type ROTATE\n", self, s->got_msgs_ROTATE);
			
        printf("                got %d messages of type MOVE_U\n",         s->got_msgs_MOVE_U);
		printf("                got %d messages of type MOVE_D\n",         s->got_msgs_MOVE_D);
		printf("                got %d messages of type MOVE_L\n",         s->got_msgs_MOVE_L);
		printf("                got %d messages of type MOVE_R\n",         s->got_msgs_MOVE_R);
        printf("                got %d messages of type BOX_GRAB\n",     s->got_msgs_BOX_GRAB);
        printf("                got %d messages of type BOX_DROP\n",     s->got_msgs_BOX_DROP);
        printf("                got %d messages of type RECEIVED\n",     s->got_msgs_RECEIVED);
        printf("                got %d messages of type INIT\n",         s->got_msgs_INIT);
		printf("                got %d messages of type NOP\n",          s->got_msgs_NOP);
}
