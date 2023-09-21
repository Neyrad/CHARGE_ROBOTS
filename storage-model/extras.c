#include "model.h"

#define MAX_PATH_TO_LOG_FOLDER 256

size_t step_number = 0;

void PrintMap(const char* log_folder_path)
{   
    ++step_number;
    char path[MAX_PATH_TO_LOG_FOLDER];
    sprintf(path, "%s/STEP_%lu.csv", log_folder_path, step_number);
    //printf("path = %s\n", path);

    FILE* f = fopen(path, "w");
    for (int y = 0; y < storage.height; ++y) {
        for (int x = 0; x < storage.length; ++x) {
            fprintf(f, "%d", storage.data[y][x]);
            if (x < storage.length - 1) fprintf(f, ","); //no comma at the end of a line
        }
        fprintf(f, "\n");
    }
    fclose(f);
}

void PrintNSteps(const char* log_folder_path)
{
    char path[MAX_PATH_TO_LOG_FOLDER];
    sprintf(path, "%s/_N_STEPS.txt", log_folder_path);  
    FILE* f = fopen(path, "w");
    fprintf(f, "%lu", step_number);
    fclose(f);
}

//Converts a CSV file to a 2D int array
void parse(const char* path)
{
    struct stat stats;
    stat(path, &stats);
    FILE* f = fopen(path, "r");
    assert(f); //to check if the path is correct
    char buf[MAX_ROOM_HEIGHT * MAX_ROOM_LENGTH * 2 + MAX_COMMENT_LENGTH]; // * 2 because of commas
    fread(buf, sizeof(buf[0]), stats.st_size, f);
    fclose(f);

    int start_index = 0;
    if (buf[0] == '#') {
        for (start_index = 0; buf[start_index] != '\n' ; ++start_index)
            ;
        ++start_index; //next symbol after #bla-bla-bla\n
    }
    
    storage.height = 0;
    int x = 0;
    for (int i = start_index; i < stats.st_size; ++i) {
        if (buf[i] == '\n') {
            ++storage.height;
            storage.length = x;
            x = 0;
            continue;
        }
        if (isdigit(buf[i])) {
            if ((i < stats.st_size) && isdigit(buf[i+1])) {
                storage.data[storage.height][x] = (buf[i] - '0') * 10 + (buf[i+1] - '0');
                ++i;
            }
            else {
                storage.data[storage.height][x] = buf[i] - '0';
            }
            ++x;
        }
    }
}

void RobotsInit()
{
    Robots.N = 0;
    for (int y = 0; y < storage.height; ++y)
        for (int x = 0; x < storage.length; ++x)
            if (storage.data[y][x] == CELL_ROBOT_UP    || \
                storage.data[y][x] == CELL_ROBOT_DOWN  || \
                storage.data[y][x] == CELL_ROBOT_LEFT  || \
                storage.data[y][x] == CELL_ROBOT_RIGHT    ) {
                
                Robots.data[Robots.N].x = x;
                Robots.data[Robots.N].y = y;

                Robots.data[Robots.N].carries_box = FALSE;
                
                switch(storage.data[y][x])
                {
                    case CELL_ROBOT_UP:
                        Robots.data[Robots.N].direction = UP;
                        break;
                    case CELL_ROBOT_DOWN:
                        Robots.data[Robots.N].direction = DOWN;
                        break;
                    case CELL_ROBOT_LEFT:
                        Robots.data[Robots.N].direction = LEFT;
                        break;
                    case CELL_ROBOT_RIGHT:
                        Robots.data[Robots.N].direction = RIGHT;
                        break;
                }
                ++Robots.N;
                assert(Robots.N <= MAX_ROBOTS);
            }
}

void RobotsPrint()
{
    for (int i = 0; i < Robots.N; ++i) {
        printf("Robot #%d is located at (%d, %d) and is facing ", \
                      i+1, Robots.data[i].x, Robots.data[i].y);
        
        switch(Robots.data[i].direction)
        {
            case UP:
                printf("UP\n");
                break;
            case DOWN:
                printf("DOWN\n");
                break;
            case LEFT:
                printf("LEFT\n");
                break;
            case RIGHT:
                printf("RIGHT\n");
                break;
        } 
    }
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
    SendMessageContents(receiver, lp, ts, type, tw_rand_unif(lp->rng));
}

int EveryoneResponded(int* arr, int N)
{
	int cnt = 0;
	for (int i = 0; i < N; ++i)
		if(arr[i])
			++cnt;
	return cnt == N;
}

void AssignDest(struct _robot* robot, int goal)
{
	if      (goal == CELL_BOX)
	{
		switch(rand() % 2)
		{
			case 0:
				{
				struct cell dest_cell = {3, 2, CELL_BOX}; // {x, y, value}
				robot->dest = dest_cell;
				robot->dest_dir = UP;
				}
				return;
			case 1:
				{
				struct cell dest_cell = {7, 2, CELL_BOX};
				robot->dest = dest_cell;
				robot->dest_dir = UP;
				}
				return;
			default:
				printf("AssignDest(): poor RNG\n");
				return;
		}
	}
	else if (goal == CELL_CONTAINER)
	{
		switch(rand() % 3)
		{
			case 0:
				{
				struct cell dest_cell = {2, 7, CELL_CONTAINER};
				robot->dest = dest_cell;
				robot->dest_dir = DOWN;
				}
				return;
			case 1:
				{
				struct cell dest_cell = {5, 7, CELL_CONTAINER};
				robot->dest = dest_cell;
				robot->dest_dir = DOWN;
				}
				return;
			case 2:
				{
				struct cell dest_cell = {8, 7, CELL_CONTAINER};
				robot->dest = dest_cell;
				robot->dest_dir = DOWN;
				}
				return;
			default:
				printf("AssignDest(): poor RNG\n");
				return;
		}
	}
}

int CalcNextMove(struct _robot* robot)
{
	
	if (robot->stuck && NeighborAlsoStuck(robot))
		return unstuck(robot);
	
	if (robot->unstucking)
	{
		robot->unstucking = FALSE;
		return MOVE;
	}
	
	//firstly y
	int dist_Y = robot->dest.y - robot->y;
	dir desired = dist_Y > 0 ? DOWN : dist_Y < 0 ? UP : NONE;
	
	//then x
	int dist_X = robot->dest.x - robot->x;
	if (desired == NONE) //y is ok
		desired = dist_X > 0 ? RIGHT : dist_X < 0 ? LEFT : NONE;
		
	
	if (desired == NONE) //already in the destination
	{
		if (robot->direction == robot->dest_dir)
			return robot->dest.value == CELL_BOX ? BOX_GRAB : BOX_DROP;
		return Rotate(robot->direction, robot->dest_dir);
	}
	
	if (robot->direction != desired)
		return Rotate(robot->direction, desired);
	
	return MOVE;
}

int Rotate(int cur, int des)
{
	switch (cur)
	{
		case UP:
			if      (des == UP)    return NONE;
			else if (des == DOWN)  return ROTATE_LEFT;
			else if (des == RIGHT) return ROTATE_RIGHT;
			else if (des == LEFT)  return ROTATE_LEFT;
			else printf("Rotate(): Unrecognized desired direction\n");
			break;
		case DOWN:
			if      (des == UP)    return ROTATE_LEFT;
			else if (des == DOWN)  return NONE;
			else if (des == RIGHT) return ROTATE_LEFT;
			else if (des == LEFT)  return ROTATE_RIGHT;
			else printf("Rotate(): Unrecognized desired direction\n");
			break;
		case RIGHT:
			if      (des == UP)    return ROTATE_LEFT;
			else if (des == DOWN)  return ROTATE_RIGHT;
			else if (des == RIGHT) return NONE;
			else if (des == LEFT)  return ROTATE_LEFT;
			else printf("Rotate(): Unrecognized desired direction\n");
			break;
		case LEFT:
			if      (des == UP)    return ROTATE_RIGHT;
			else if (des == DOWN)  return ROTATE_LEFT;
			else if (des == RIGHT) return ROTATE_LEFT;
			else if (des == LEFT)  return NONE;
			else printf("Rotate(): Unrecognized desired direction\n");
			break;
		default:
			printf("Rotate(): Unrecognized current direction\n");
			return NONE;
	}
}

int unstuck(struct _robot* me)
{
	me->stuck      = FALSE;
	me->unstucking = TRUE;
	
	struct cell up	  = {me->x  , me->y-1, storage.data[me->y-1][me->x  ]};
	struct cell down  = {me->x  , me->y+1, storage.data[me->y+1][me->x  ]};
	struct cell left  = {me->x-1, me->y  , storage.data[me->y  ][me->x-1]};
	struct cell right = {me->x+1, me->y  , storage.data[me->y  ][me->x+1]};
	
	struct cell rel_left;
	struct cell rel_right;
	
	switch(me->direction)
	{
		case UP:
			rel_left  = left;
			rel_right = right;
			break;
		case DOWN:
			rel_left  = right;
			rel_right = left;
			break;
		case LEFT:
			rel_left  = down;
			rel_right = up;
			break;
		case RIGHT:
			rel_left  = up;
			rel_right = down;
			break;
	}
	
	if      (rel_left.value  == CELL_EMPTY)
		return ROTATE_LEFT;
	else if (rel_right.value == CELL_EMPTY)
		return ROTATE_RIGHT;

	me->stuck      = TRUE;
	me->unstucking = FALSE;
	return NOP;
}

bool NeighborAlsoStuck(struct _robot* This)
{
	struct cell NeigbourCoords;
	switch(This->direction)
    {
        case UP:
            NeigbourCoords.x     = This->x    ;
            NeigbourCoords.y     = This->y - 1;
            break;
        case DOWN:
            NeigbourCoords.x     = This->x    ;
            NeigbourCoords.y     = This->y + 1;
            break;
        case LEFT:
            NeigbourCoords.x     = This->x - 1;
            NeigbourCoords.y     = This->y    ;
            break;    
        case RIGHT:
            NeigbourCoords.x     = This->x + 1;
            NeigbourCoords.y     = This->y    ;
            break;
    }
	
	for (int i = 0; i < Robots.N; ++i)
		if (Robots.data[i].x == NeigbourCoords.x &&\
			Robots.data[i].y == NeigbourCoords.y)
			return Robots.data[i].stuck;
}