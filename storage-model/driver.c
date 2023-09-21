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
		
		Robots.data[lp->gid - 1].carries_box = FALSE;
		Robots.data[lp->gid - 1].state       = STOP;
		Robots.data[lp->gid - 1].stuck       = FALSE;
		Robots.data[lp->gid - 1].unstucking  = FALSE;
		AssignDest(&Robots.data[lp->gid - 1], CELL_BOX);
    
        printf("ROBOT #%ld is initialized\n", lp->gid);
    }

    int self = lp->gid;

    // init state data
    s->value = -1;

    s->got_msgs_ROTATE_LEFT          = 0;
    s->got_msgs_ROTATE_RIGHT         = 0;
    s->got_msgs_MOVE                 = 0;
    s->got_msgs_BOX_GRAB             = 0;
    s->got_msgs_BOX_DROP             = 0;
    s->got_msgs_RECEIVED             = 0;
    s->got_msgs_EXECUTED             = 0;
    s->got_msgs_INIT                 = 0;
	s->got_msgs_NOP                  = 0;

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
                case EXECUTED:
                    ++s->got_msgs_EXECUTED; 
                    break;
                case INIT:
                    ++s->got_msgs_INIT; 
                    break;
                default:
                    printf("COMMAND CENTER: Unhandled forward message type %d\n", in_msg->type);
            }

            //if (glb_time > 40) return;

			RobotResponded[in_msg->sender-1] = TRUE;
			if (EveryoneResponded(RobotResponded, Robots.N))
			{
				PrintMap(path_to_log_folder);
				glb_time += 1;
				
				for (int i = 0; i < Robots.N; ++i)
					RobotResponded[i] = FALSE;
				
				
				for (int i = 1; i <= Robots.N; ++i)
				{
					message_type cmd = CalcNextMove(&Robots.data[i-1]);
					SendMessage(i, lp, glb_time, cmd);
				}
			}
			
			

            break;

        case ROBOT:
            {
            struct _robot* This = &Robots.data[self-1];
            struct cell cell_in_front = {-1, -1, -1};
            switch (in_msg->type)
            {
                case ROTATE_LEFT:
                    ++s->got_msgs_ROTATE_LEFT; 
                    switch(This->direction)         
                    {
                        case UP:
                            This->direction = LEFT;
                            storage.data[This->y][This->x] = This->carries_box ? CELL_ROBOT_WITH_BOX_LEFT : CELL_ROBOT_LEFT;
                            break;
                        case DOWN:
                            This->direction = RIGHT;
                            storage.data[This->y][This->x] = This->carries_box ? CELL_ROBOT_WITH_BOX_RIGHT : CELL_ROBOT_RIGHT;
                            break;
                        case LEFT:
                            This->direction = DOWN;
                            storage.data[This->y][This->x] = This->carries_box ? CELL_ROBOT_WITH_BOX_DOWN : CELL_ROBOT_DOWN;
                            break;    
                        case RIGHT:
                            This->direction = UP;
                            storage.data[This->y][This->x] = This->carries_box ? CELL_ROBOT_WITH_BOX_UP : CELL_ROBOT_UP;
                            break;
                    }
                    is_executed = TRUE;
                    break;
                case ROTATE_RIGHT:
                    ++s->got_msgs_ROTATE_RIGHT;
                    switch(This->direction)  
                    {
                        case UP:
                            This->direction = RIGHT;
                            storage.data[This->y][This->x] = This->carries_box ? CELL_ROBOT_WITH_BOX_RIGHT : CELL_ROBOT_RIGHT;
                            break;
                        case DOWN:
                            This->direction = LEFT;
                            storage.data[This->y][This->x] = This->carries_box ? CELL_ROBOT_WITH_BOX_LEFT : CELL_ROBOT_LEFT;
                            break;
                        case LEFT:
                            This->direction = UP;
                            storage.data[This->y][This->x] = This->carries_box ? CELL_ROBOT_WITH_BOX_UP : CELL_ROBOT_UP;
                            break;    
                        case RIGHT:
                            This->direction = DOWN;
                            storage.data[This->y][This->x] = This->carries_box ? CELL_ROBOT_WITH_BOX_DOWN : CELL_ROBOT_DOWN;
                            break;
                    }
                    is_executed = TRUE;
                    break;
                case MOVE:
                    ++s->got_msgs_MOVE;
                    struct cell dest_cell = {-1, -1, -1};
                    switch(This->direction)         
                    {
                        case UP:
                            assert(This->y - 1 >= 0); //to stay within the boundaries of the map
                            dest_cell.x     = This->x    ;
                            dest_cell.y     = This->y - 1;
                            break;
                        case DOWN:
                            assert(This->y + 1 <= storage.height - 1);
                            dest_cell.x     = This->x    ;
                            dest_cell.y     = This->y + 1;
                            break;
                        case LEFT:
                            assert(This->x - 1 >= 0);
                            dest_cell.x     = This->x - 1;
                            dest_cell.y     = This->y    ;
                            break;    
                        case RIGHT:
                            assert(This->x + 1 <= storage.length - 1);
                            dest_cell.x     = This->x + 1;
                            dest_cell.y     = This->y    ;
                            break;
                    }

                    switch(storage.data[dest_cell.y][dest_cell.x])
                    {
                        case CELL_EMPTY:
							if (This->stuck)
								This->stuck = FALSE;
                            switch(This->direction)  
                            {
                                case UP:
                                    storage.data[dest_cell.y][dest_cell.x] = \
                                       This->carries_box ? CELL_ROBOT_WITH_BOX_UP : CELL_ROBOT_UP;
                                    break;
                                case DOWN:
                                    storage.data[dest_cell.y][dest_cell.x] = \
                                       This->carries_box ? CELL_ROBOT_WITH_BOX_DOWN : CELL_ROBOT_DOWN;
                                    break;
                                case LEFT:
                                    storage.data[dest_cell.y][dest_cell.x] = \
                                       This->carries_box ? CELL_ROBOT_WITH_BOX_LEFT : CELL_ROBOT_LEFT;
                                    break;    
                                case RIGHT:
                                    storage.data[dest_cell.y][dest_cell.x] = \
                                       This->carries_box ? CELL_ROBOT_WITH_BOX_RIGHT : CELL_ROBOT_RIGHT;
                                    break;
                            }
                            storage.data[This->y][This->x] = CELL_EMPTY;
                            This->x = dest_cell.x;
                            This->y = dest_cell.y;
                            is_executed = TRUE;
                            break;
                        case CELL_WALL:
							break;
						case CELL_BOX:
                            break;
                        case CELL_CONTAINER:
                            break;
                        case CELL_ROBOT_WITH_BOX_UP:
							if (This->direction == DOWN)
								This->stuck = TRUE;
							break;
                        case CELL_ROBOT_WITH_BOX_DOWN:
							if (This->direction == UP)
								This->stuck = TRUE;
							break;
                        case CELL_ROBOT_WITH_BOX_LEFT:
                            if (This->direction == RIGHT)
								This->stuck = TRUE;
							break;
                        case CELL_ROBOT_WITH_BOX_RIGHT:
                            if (This->direction == LEFT)
								This->stuck = TRUE;
							break;
                        case CELL_ROBOT_UP:
                            if (This->direction == DOWN)
								This->stuck = TRUE;
							break;
                        case CELL_ROBOT_DOWN:
                            if (This->direction == UP)
								This->stuck = TRUE;
							break;
                        case CELL_ROBOT_LEFT:
                            if (This->direction == RIGHT)
								This->stuck = TRUE;
							break;
                        case CELL_ROBOT_RIGHT:
                            if (This->direction == LEFT)
								This->stuck = TRUE;
							break;
                        default:
                            break;
                    }

                    break;
                case BOX_GRAB:
                    ++s->got_msgs_BOX_GRAB;
                    switch(This->direction)         
                    {
                        case UP:
                            assert(This->y - 1 >= 0); //to stay within the boundaries of the map
                            cell_in_front.x     = This->x    ;
                            cell_in_front.y     = This->y - 1;
                            break;
                        case DOWN:
                            assert(This->y + 1 <= storage.height - 1);
                            cell_in_front.x     = This->x    ;
                            cell_in_front.y     = This->y + 1;
                            break;
                        case LEFT:
                            assert(This->x - 1 >= 0);
                            cell_in_front.x     = This->x - 1;
                            cell_in_front.y     = This->y    ;
                            break;    
                        case RIGHT:
                            assert(This->x + 1 <= storage.length - 1);
                            cell_in_front.x     = This->x + 1;
                            cell_in_front.y     = This->y    ;
                            break;
                    }

                    cell_in_front.value = storage.data[cell_in_front.y][cell_in_front.x];

                    if (cell_in_front.value == CELL_BOX && This->carries_box == FALSE)
                    {
                        This->carries_box = TRUE;
                        switch(This->direction)
                        {
                            case UP:
                                storage.data[This->y][This->x] = CELL_ROBOT_WITH_BOX_UP;
                                break;
                            case DOWN:
                                storage.data[This->y][This->x] = CELL_ROBOT_WITH_BOX_DOWN;
                                break;
                            case LEFT:
                                storage.data[This->y][This->x] = CELL_ROBOT_WITH_BOX_LEFT;
                                break;
                            case RIGHT:
                                storage.data[This->y][This->x] = CELL_ROBOT_WITH_BOX_RIGHT;
                                break;
                        }
                        printf("ROBOT #%d: grab the box.\n", self);
                        //storage.data[cell_in_front.y][cell_in_front.x] = CELL_WALL;
						AssignDest(&Robots.data[self-1], CELL_CONTAINER);
                        is_executed = TRUE;

                    }
                    break;
                case BOX_DROP:
                    ++s->got_msgs_BOX_DROP;
                    switch(This->direction)         
                    {
                        case UP:
                            assert(This->y - 1 >= 0); //to stay within the boundaries of the map
                            cell_in_front.x     = This->x    ;
                            cell_in_front.y     = This->y - 1;
                            break;
                        case DOWN:
                            assert(This->y + 1 <= storage.height - 1);
                            cell_in_front.x     = This->x    ;
                            cell_in_front.y     = This->y + 1;
                            break;
                        case LEFT:
                            assert(This->x - 1 >= 0);
                            cell_in_front.x     = This->x - 1;
                            cell_in_front.y     = This->y    ;
                            break;    
                        case RIGHT:
                            assert(This->x + 1 <= storage.length - 1);
                            cell_in_front.x     = This->x + 1;
                            cell_in_front.y     = This->y    ;
                            break;
                    }

                    cell_in_front.value = storage.data[cell_in_front.y][cell_in_front.x];

                    if (cell_in_front.value == CELL_CONTAINER && This->carries_box == TRUE)
                    {
                        This->carries_box = FALSE;
                        switch(This->direction)
                        {
                            case UP:
                                storage.data[This->y][This->x] = CELL_ROBOT_UP;
                                break;
                            case DOWN:
                                storage.data[This->y][This->x] = CELL_ROBOT_DOWN;
                                break;
                            case LEFT:
                                storage.data[This->y][This->x] = CELL_ROBOT_LEFT;
                                break;
                            case RIGHT:
                                storage.data[This->y][This->x] = CELL_ROBOT_RIGHT;
                                break;
                        }
						printf("ROBOT #%d: drop the box.\n", self);
                        //storage.data[cell_in_front.y][cell_in_front.x] = CELL_BOX;
						AssignDest(&Robots.data[self-1], CELL_BOX);
                        is_executed = TRUE;

                    }
                    break;
                case INIT:
                    ++s->got_msgs_INIT; 
                    break;
				case NOP:
					++s->got_msgs_NOP;
					break;
                default:
                    printf("ROBOT #%d: Unhandled forward message of type %d\n", self, in_msg->type);
            }

            if (in_msg->sender == 0) //the message came from the command center
            {
                SendMessage(0, lp, glb_time, RECEIVED);

                if (is_executed == TRUE)
                {
                    //SendMessage(0, lp, glb_time, EXECUTED);
                    //++s->sent_msgs_EXECUTED;
                }
            }

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
        printf("COMMAND_CENTER: got %d messages of type ROTATE_LEFT\n",        s->got_msgs_ROTATE_LEFT);
	else if (s->type == ROBOT)
		printf("ROBOT #%2d:      got %d messages of type ROTATE_LEFT\n", self, s->got_msgs_ROTATE_LEFT);
			
        printf("                got %d messages of type ROTATE_RIGHT\n", s->got_msgs_ROTATE_RIGHT);
        printf("                got %d messages of type MOVE\n",         s->got_msgs_MOVE);
        printf("                got %d messages of type BOX_GRAB\n",     s->got_msgs_BOX_GRAB);
        printf("                got %d messages of type BOX_DROP\n",     s->got_msgs_BOX_DROP);
        printf("                got %d messages of type RECEIVED\n",     s->got_msgs_RECEIVED);
        printf("                got %d messages of type EXECUTED\n",     s->got_msgs_EXECUTED);
        printf("                got %d messages of type INIT\n",         s->got_msgs_INIT);
		printf("                got %d messages of type NOP\n",          s->got_msgs_NOP);
}
