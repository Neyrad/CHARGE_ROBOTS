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

unsigned int setting_1 = 0;

const tw_optdef model_opts[] = {
    TWOPT_GROUP("ROSS Model"),
    TWOPT_UINT("setting_1", setting_1, "first setting for this model"),
    TWOPT_END(),
};

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

extern const char* path_to_log_folder;

int glb_time = 0;
int RobotResponded[MAX_ROBOTS];

//Init function
// - called once for each LP
// ! LP can only send messages to itself during init !
void model_init(state* s, tw_lp* lp)
{
    if (lp->gid == 0)
    {
        s->type = COMMAND_CENTER;
		
		for (int i = 0; i < Robots.N; ++i)
		{
			RobotResponded[i] = false;
			InitBDM(&Robots.elem[i], (i == 0)? LiFePO4: (i == 1)? LiNiMnCoO2: (i == 2)? LeadAcid: LiCoO2);
			//PrintBDM(&Robots.elem[i], (i == 0)? "graph/LiFePO4.csv": (i == 1)? "graph/LiNiMnCoO2.csv": (i == 2)? "graph/LeadAcid.csv": "graph/LiCoO2.csv");
		}
		
        printf("COMMAND_CENTER is initialized\n");
    }
    else
    {
        s->type = ROBOT;
        assert(lp->gid <= Robots.N);
		
		Robots.elem[lp->gid - 1].state          	 		 = STOP;
		Robots.elem[lp->gid - 1].battery.charge   		     = BATTERY_CAPACITY;
		Robots.elem[lp->gid - 1].battery.capacity        	 = BATTERY_CAPACITY;
		Robots.elem[lp->gid - 1].battery.charging        	 = false;
		//Robots.elem[lp->gid - 1].battery.dead        	 	 = false;
		Robots.elem[lp->gid - 1].time_in_action  			 = 0; //no commands received, no actions performed
		Robots.elem[lp->gid - 1].battery.times_recharged 	 = 0;
		Robots.elem[lp->gid - 1].battery.time_spent_charging = 0;
		Robots.elem[lp->gid - 1].boxes_delivered 			 = 0;
		Robots.elem[lp->gid - 1].stuck 						 = 0;
		
		Robots.elem[lp->gid - 1].next_move_l 				 = false;
		Robots.elem[lp->gid - 1].next_move_r 				 = false;
		Robots.elem[lp->gid - 1].next_move_u 				 = false;
		Robots.elem[lp->gid - 1].next_move_d 				 = false;
		Robots.elem[lp->gid - 1].next_move_stall			 = false;
		
		AssignDest(&Robots.elem[lp->gid - 1], CELL_IN);
    
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
    s->got_msgs_LOAD	 = 0;
    s->got_msgs_UNLOAD 	 = 0;
    s->got_msgs_RECEIVED = 0;
    s->got_msgs_INIT     = 0;
	s->got_msgs_NOP      = 0;
	
    if (lp->gid == 0)
		for (int i = 1; i <= Robots.N; ++i)
			SendMessage(i, lp, glb_time, INIT);
}

//Forward event handler
void model_event(state* s, tw_bf* bf, message* in_msg, tw_lp* lp)
{
    int self = lp->gid;
    bool is_executed = false;
    // initialize the bit field
    *(int*)bf = (int)0;

    // update the current state
    // however, save the old value in the 'reverse' message
    SWAP(&(s->value), &(in_msg->contents));

    // handle the message
    switch(s->type)
    {
        case COMMAND_CENTER:
            switch (in_msg->type)
            {
                case RECEIVED:
                    ++s->got_msgs_RECEIVED; 
                    break;
					/*
				case DEAD;
					++s->got_msgs_DEAD;
					BatteryDeath(&Robots.elem[in_msg->sender - 1]);
					break;
					*/
                default:
                    printf("COMMAND CENTER: Unhandled forward message type %d\n", in_msg->type);
            }

            if (pairs.eof || glb_time >= GLOBAL_TIME_END)
				return;
			
			for (int i = 0; i < Robots.N; ++i)
				if (Robots.elem[i].battery.BDM_cur >= MAX_CYCLES_LiFePO4    && Robots.elem[i].battery.type == LiFePO4 || \
					Robots.elem[i].battery.BDM_cur >= MAX_CYCLES_LiNiMnCoO2 && Robots.elem[i].battery.type == LiNiMnCoO2 || \
					Robots.elem[i].battery.BDM_cur >= MAX_CYCLES_LeadAcid   && Robots.elem[i].battery.type == LeadAcid || \
					Robots.elem[i].battery.BDM_cur >= MAX_CYCLES_LiCoO2     && Robots.elem[i].battery.type == LiCoO2)
						return;


			RobotResponded[in_msg->sender-1] = true;
			if (EveryoneResponded(RobotResponded, Robots.N))
			{
				//if (glb_time % 9000 == 0) // every hour
				//	PrintNBoxesDelivered();
				
				PrintMap(path_to_log_folder);
				glb_time += 1;
				
				for (int i = 0; i < Robots.N; ++i)
					RobotResponded[i] = false;
				
				
				for (int i = 1; i <= Robots.N; ++i)
				{
					message_type cmd = CalcNextMove(&Robots.elem[i-1]);
					SendMessage(i, lp, glb_time, cmd);
				}
			}
            break;

        case ROBOT:
            {
            struct _robot* this = &Robots.elem[self-1];
			
			switch (in_msg->type)
            {
                case ROTATE:
                    ++s->got_msgs_ROTATE;
					if (this->time_in_action > 1) //busy
						break;
					this->time_in_action = ROTATE_TIME;
					
					if (this->state == MOTION)
						this->battery.charge -= STOP_MOTION_COST;
					this->state = STOP;
					
					if (this->cur_ori == VER)
					{
						this->cur_ori = HOR;
						warehouse.robots[this->y][this->x] = this->loaded? CELL_FULL_ROBOT_HOR: CELL_EMPT_ROBOT_HOR;
					}
					else
					{
						this->cur_ori = VER;
						warehouse.robots[this->y][this->x] = this->loaded? CELL_FULL_ROBOT_VER: CELL_EMPT_ROBOT_VER;
					}
					
					this->battery.charge -= ROTATE_COST;
                    break;
                case MOVE_U:
                    ++s->got_msgs_MOVE_U;
					if (this->time_in_action > 1)
						break;
					this->time_in_action = MOVE_TIME;
					
					assert(this->cur_ori == VER);
					assert(this->y - 1 >= 0);
                    switch(warehouse.room[this->y - 1][this->x])
                    {
                        case CELL_EMPTY:
						case CELL_IN:
						case CELL_OUT:
						case CELL_CHARGER:
							if (warehouse.robots[this->y - 1][this->x] == CELL_EMPTY)
							{
								warehouse.robots[this->y - 1][this->x] = this->loaded? CELL_FULL_ROBOT_VER: CELL_EMPT_ROBOT_VER;
								warehouse.robots[this->y    ][this->x] = CELL_EMPTY;
								this->y = this->y - 1;
								
								if 		(this->state == MOTION)
									this->battery.charge -= KEEP_MOTION_COST;
								else if (this->state == STOP)
									this->battery.charge -= START_MOTION_COST;
								
								this->state = MOTION;
								this->stuck = 0;
							}
							else //the way is blocked by another robot
							{
								++this->stuck;
								printf("MOVE_U: incrementing this->stuck...\n");
							}
							break;
                        case CELL_WALL:
							this->state = STOP;
							break;
                        default:
                            break;
                    }
                    break;
				case MOVE_D:
                    ++s->got_msgs_MOVE_D;
					if (this->time_in_action > 1)
						break;
					this->time_in_action = MOVE_TIME;
					
					assert(this->cur_ori == VER);
					assert(this->y + 1 < warehouse.size_y);
                    switch(warehouse.room[this->y + 1][this->x])
                    {
                        case CELL_EMPTY:
						case CELL_IN:
						case CELL_OUT:
						case CELL_CHARGER:
							if (warehouse.robots[this->y + 1][this->x] == CELL_EMPTY)
							{
								warehouse.robots[this->y + 1][this->x] = this->loaded? CELL_FULL_ROBOT_VER: CELL_EMPT_ROBOT_VER;
								warehouse.robots[this->y    ][this->x] = CELL_EMPTY;
								this->y = this->y + 1;
								
								if 		(this->state == MOTION)
									this->battery.charge -= KEEP_MOTION_COST;
								else if (this->state == STOP)
									this->battery.charge -= START_MOTION_COST;
								
								this->state = MOTION;
								this->stuck = 0;
							}
							else //the way is blocked by another robot
							{
								++this->stuck;
								printf("MOVE_D: incrementing this->stuck...\n");
							}
                            break;
                        case CELL_WALL:
							this->state = STOP;
							break;
                        default:
                            break;
                    }
                    break;
				case MOVE_L:
                    ++s->got_msgs_MOVE_L;
					if (this->time_in_action > 1)
						break;
					this->time_in_action = MOVE_TIME;
					
					assert(this->cur_ori == HOR);
					assert(this->x - 1 >= 0);
                    switch(warehouse.room[this->y][this->x - 1])
                    {
                        case CELL_EMPTY:
						case CELL_IN:
						case CELL_OUT:
						case CELL_CHARGER:
							if (warehouse.robots[this->y][this->x - 1] == CELL_EMPTY)
							{
								warehouse.robots[this->y][this->x - 1] = this->loaded? CELL_FULL_ROBOT_HOR: CELL_EMPT_ROBOT_HOR;
								warehouse.robots[this->y][this->x    ] = CELL_EMPTY;
								this->x = this->x - 1;
								
								if 		(this->state == MOTION)
									this->battery.charge -= KEEP_MOTION_COST;
								else if (this->state == STOP)
									this->battery.charge -= START_MOTION_COST;
								
								this->state = MOTION;
								this->stuck = 0;
							}
							else //the way is blocked by another robot
							{
								++this->stuck;
								printf("MOVE_L: incrementing this->stuck...\n");
							}
                            break;
                        case CELL_WALL:
							this->state = STOP;
							break;
                        default:
                            break;
                    }
                    break;
				case MOVE_R:
                    ++s->got_msgs_MOVE_R;
					if (this->time_in_action > 1)
						break;
					this->time_in_action = MOVE_TIME;
					
					assert(this->cur_ori == HOR);
					//printf("this->x = %d, warehouse.size_x = %d\n", this->x, warehouse.size_x);
					assert(this->x + 1 < warehouse.size_x);
                    switch(warehouse.room[this->y][this->x + 1])
                    {
                        case CELL_EMPTY:
						case CELL_IN:
						case CELL_OUT:
						case CELL_CHARGER:
							if (warehouse.robots[this->y][this->x + 1] == CELL_EMPTY)
							{
								warehouse.robots[this->y][this->x + 1] = this->loaded? CELL_FULL_ROBOT_HOR: CELL_EMPT_ROBOT_HOR;
								warehouse.robots[this->y][this->x]     = CELL_EMPTY;
								this->x = this->x + 1;
								
								if 		(this->state == MOTION)
									this->battery.charge -= KEEP_MOTION_COST;
								else if (this->state == STOP)
									this->battery.charge -= START_MOTION_COST;
								
								this->state = MOTION;
								this->stuck = 0;
							}
							else //the way is blocked by another robot
							{
								++this->stuck;
								printf("MOVE_R: incrementing this->stuck...\n");
							}
                            break;
                        case CELL_WALL:
							this->state = STOP;
							break;
                        default:
                            break;
                    }
                    break;
                case LOAD:
                    ++s->got_msgs_LOAD;
					if (this->time_in_action > 1)
						break;
					this->time_in_action = LOAD_TIME;
					
					if (this->state == MOTION)
						this->battery.charge -= STOP_MOTION_COST;
					this->state = STOP;
					
                    if (warehouse.room[this->y][this->x] == CELL_IN && this->loaded == false)
                    {
                        this->loaded = true;
						if (this->cur_ori == VER)
							warehouse.robots[this->y][this->x] = CELL_FULL_ROBOT_VER;
						else
							warehouse.robots[this->y][this->x] = CELL_FULL_ROBOT_HOR;
						
						AssignDest(&Robots.elem[self-1], CELL_OUT);
                    }
                    break;
                case UNLOAD:
                    ++s->got_msgs_UNLOAD;
					if (this->time_in_action > 1)
						break;
					this->time_in_action = UNLOAD_TIME;
					
					if (this->state == MOTION)
						this->battery.charge -= STOP_MOTION_COST;
					this->state = STOP;
					
                    if (warehouse.room[this->y][this->x] == CELL_OUT && this->loaded == true)
                    {
                        this->loaded = false;
						if (this->cur_ori == VER)
							warehouse.robots[this->y][this->x] = CELL_EMPT_ROBOT_VER;
						else
							warehouse.robots[this->y][this->x] = CELL_EMPT_ROBOT_HOR;
                        
						++this->boxes_delivered;
						
						//if (Robots.elem[self-1].battery.charge < TIME_TO_CHARGE_THRESHOLD)
						//	AssignDest(&Robots.elem[self-1], CELL_CHARGER);
						//else
							AssignDest(&Robots.elem[self-1], CELL_IN);
                    }
                    break;
				case INIT:
					++s->got_msgs_INIT;
					break;
				case NOP:
					++s->got_msgs_NOP;
					if (this->state == MOTION)
						this->battery.charge -= STOP_MOTION_COST;
					this->state = STOP;
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
void model_event_reverse(state* s, tw_bf* bf, message* in_msg, tw_lp* lp)
{
	return;
}

//report any final statistics for this LP
void model_final(state* s, tw_lp* lp)
{
    int self = lp->gid;
    if      (s->type == COMMAND_CENTER)
	{
        /*
		printf("COMMAND_CENTER:\n");
		printf("                got %4d messages of type RECEIVED\n",   	s->got_msgs_RECEIVED);
		*/
	}
	else if (s->type == ROBOT)
	{
		printf("\nROBOT #%d (battery %d/%d) ", self, Robots.elem[self-1].battery.charge, Robots.elem[self-1].battery.capacity);
		switch(Robots.elem[self-1].battery.type)
		{
			case LiFePO4:
				printf("LiFePO4:\n\n");
				break;
			case LiNiMnCoO2:
				printf("LiNiMnCoO2:\n\n");
				break;
			case LeadAcid:
				printf("LeadAcid:\n\n");
				break;
			case LiCoO2:
				printf("LiCoO2:\n\n");
				break;
			default:
				printf("UNKNOWN:\n\n");
				break;
		}
		
		/*
		printf("                got %8d messages of type ROTATE\n", 		s->got_msgs_ROTATE);
		printf("                got %8d messages of type MOVE_U\n",         s->got_msgs_MOVE_U);
		printf("                got %8d messages of type MOVE_D\n",         s->got_msgs_MOVE_D);
		printf("                got %8d messages of type MOVE_L\n",         s->got_msgs_MOVE_L);
		printf("                got %8d messages of type MOVE_R\n",         s->got_msgs_MOVE_R);
		printf("                got %8d messages of type LOAD\n",       s->got_msgs_LOAD);
		printf("                got %8d messages of type UNLOAD\n",       s->got_msgs_UNLOAD);
		printf("                got %8d messages of type INIT\n",           s->got_msgs_INIT);
		printf("                got %8d messages of type NOP\n",            s->got_msgs_NOP);
		*/
		
		printf("                delivered %8d boxes\n",			            Robots.elem[self-1].boxes_delivered);
		printf("                recharged %8d times\n",			            Robots.elem[self-1].battery.times_recharged);
		printf("                battery capacity loss %d%%\n", \
								(int)( (1 - (float)Robots.elem[self-1].battery.capacity / (float)BATTERY_CAPACITY) * 100 ));
		
		printf("                time spent charging   %d days\n", Robots.elem[lp->gid - 1].battery.time_spent_charging / (9000 * 24)); // 1 tick == 0.5 sec
		printf("                time spent uncharging %d days\n", (glb_time - Robots.elem[lp->gid - 1].battery.time_spent_charging) / (9000 * 24));
		
		if (Robots.elem[self-1].battery.times_recharged != 0)
		{
			printf("                avg work time from 1 charge %d min\n", (glb_time - Robots.elem[lp->gid - 1].battery.time_spent_charging)\
																		/ Robots.elem[self-1].battery.times_recharged / 150);
			printf("                avg time on charge          %d min\n", Robots.elem[lp->gid - 1].battery.time_spent_charging \
																		/ Robots.elem[self-1].battery.times_recharged / 150);
		}
	}
}
