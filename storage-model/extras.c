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
            fprintf(f, "%d", storage.robots[y][x]);
            if (x < storage.length - 1)
				fprintf(f, ","); //no comma at the end of a line
        }
        fprintf(f, "\n");
    }
    fclose(f);
}

void PrintPairs()
{
	for (int y = 0; y < pairs.length; ++y)
	{
        for (int x = 0; x < 2; ++x)
		{
            printf("%d", pairs.data[y][x]);
            if (x == 0)
				printf(","); //no comma at the end of a line
        }
        printf("\n");
    }
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
void Parse(const char* path, int arr[MAX_ROOM_HEIGHT][MAX_ROOM_LENGTH])
{
    struct stat stats;
    stat(path, &stats);
    FILE* f = fopen(path, "r");
    assert(f); //to check correctness of the path
    char buf[MAX_ROOM_HEIGHT * MAX_ROOM_LENGTH * 2 + MAX_COMMENT_LENGTH]; // * 2 for commas
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
            if ((i < stats.st_size) && isdigit(buf[i+1])) {  //processing 2-digit numbers
                arr[storage.height][x] = (buf[i] - '0') * 10 + (buf[i+1] - '0');
                ++i;
            }
            else {
                arr[storage.height][x] = buf[i] - '0';
            }
            ++x;
        }
    }
}

void ParsePairs(const char* path)
{
    struct stat stats;
    stat(path, &stats);
    FILE* f = fopen(path, "r");
    assert(f); //to check correctness of the path
    char buf[MAX_INPUT_LENGTH * 4]; //'1' ',' '2' '\n' -> 4 columns
    fread(buf, sizeof(buf[0]), stats.st_size, f);
    fclose(f);

	pairs.length = 0;
    int x = 0;
    for (int i = 0; i < stats.st_size; ++i)
	{
        if (buf[i] == '\n')
		{
            ++pairs.length;
            x = 0;
            continue;
        }
        if (isdigit(buf[i]))
		{
            pairs.data[pairs.length][x] = buf[i] - '0';
            ++x;
        }
    }
	pairs.cur = 0;
	pairs.eof = FALSE;
}

void RobotsInit()
{
    Robots.N = 0;
    for (int y = 0; y < storage.height; ++y)
        for (int x = 0; x < storage.length; ++x)
            if (storage.robots[y][x] == CELL_ROBOT_VER   		|| \
                storage.robots[y][x] == CELL_ROBOT_HOR   		|| \
				storage.robots[y][x] == CELL_ROBOT_WITH_BOX_VER || \
                storage.robots[y][x] == CELL_ROBOT_WITH_BOX_HOR) 
			{
				if (storage.robots[y][x] == CELL_ROBOT_VER || \
					storage.robots[y][x] == CELL_ROBOT_HOR)
					Robots.data[Robots.N].carries_box = FALSE;
				else
					Robots.data[Robots.N].carries_box = TRUE;
				
                Robots.data[Robots.N].x = x;
                Robots.data[Robots.N].y = y;
                
                if (storage.robots[y][x] == CELL_ROBOT_VER || \
				    storage.robots[y][x] == CELL_ROBOT_WITH_BOX_VER)
				    Robots.data[Robots.N].orientation = VER;
				else
					Robots.data[Robots.N].orientation = HOR;

                ++Robots.N;
                assert(Robots.N <= MAX_ROBOTS);
            }
}

void RobotsPrint()
{
    for (int i = 0; i < Robots.N; ++i)
	{
        printf("Robot #%d is located at (%d, %d) ", \
                      i+1, Robots.data[i].x, Robots.data[i].y);
        if (Robots.data[i].orientation == VER)
			printf("VER\n");
		else
			printf("HOR\n");
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
		GetPair(robot);
		switch(robot->pair[0]) //instead of random choice read BOX -> CONTAINER pair from the log file
		{
			case 1:
				{
				struct cell dest_cell = {2, 7, CELL_BOX}; // {x, y, value}
				robot->dest = dest_cell;
				robot->dest_ori = VER;
				}
				return;
			case 2:
				{
				struct cell dest_cell = {4, 7, CELL_BOX};
				robot->dest = dest_cell;
				robot->dest_ori = VER;
				}
				return;
			case 3:
				{
				struct cell dest_cell = {6, 7, CELL_BOX};
				robot->dest = dest_cell;
				robot->dest_ori = VER;
				}
				return;
			default:
				printf("AssignDest(): poor RNG\n");
				return;
		}
	}
	else if (goal == CELL_CONTAINER)
	{
		switch(robot->pair[1]) //should already know based on the box number (box - contatiner pair from the log file)
		{
			case 1:
				{
				struct cell dest_cell = {0, 6, CELL_CONTAINER};
				robot->dest = dest_cell;
				robot->dest_ori = HOR;
				}
				return;
			case 2:
				{
				struct cell dest_cell = {0, 4, CELL_CONTAINER};
				robot->dest = dest_cell;
				robot->dest_ori = HOR;
				}
				return;
			case 3:
				{
				struct cell dest_cell = {0, 2, CELL_CONTAINER};
				robot->dest = dest_cell;
				robot->dest_ori = HOR;
				}
				return;
			case 4:
				{
				struct cell dest_cell = {2, 0, CELL_CONTAINER};
				robot->dest = dest_cell;
				robot->dest_ori = VER;
				}
				return;
			case 5:
				{
				struct cell dest_cell = {4, 0, CELL_CONTAINER};
				robot->dest = dest_cell;
				robot->dest_ori = VER;
				}
				return;
			case 6:
				{
				struct cell dest_cell = {6, 0, CELL_CONTAINER};
				robot->dest = dest_cell;
				robot->dest_ori = VER;
				}
				return;
			case 7:
				{
				struct cell dest_cell = {8, 6, CELL_CONTAINER};
				robot->dest = dest_cell;
				robot->dest_ori = HOR;
				}
				return;
			case 8:
				{
				struct cell dest_cell = {8, 4, CELL_CONTAINER};
				robot->dest = dest_cell;
				robot->dest_ori = HOR;
				}
				return;
			case 9:
				{
				struct cell dest_cell = {8, 2, CELL_CONTAINER};
				robot->dest = dest_cell;
				robot->dest_ori = HOR;
				}
				return;
			default:
				printf("AssignDest(): poor RNG\n");
				return;
		}
	}
	
	else if (goal == CELL_CHARGER)
	{
		//choose the closest free charger
		if (robot->y == 0)
		{
			struct cell dest_cell = {8, 0, CELL_CHARGER};
			robot->dest = dest_cell;
			robot->dest_ori = VER;
		}	
		else
		{
			struct cell dest_cell = {0, 0, CELL_CHARGER};
			robot->dest = dest_cell;
			robot->dest_ori = VER;
		}
	}
}

/*
int CalcNextMove(struct _robot* robot)
{
	int dist_X = robot->dest.x - robot->x;
	int dist_Y = robot->dest.y - robot->y;
	
	if (dist_Y != 0)
	{
		if (robot->orientation == HOR)
			return ROTATE;
		
		if (dist_Y > 0)
			return MOVE_D;
		else
			return MOVE_U;
	}
	
	if (dist_X != 0)
	{
		if (robot->orientation == VER)
			return ROTATE;
		
		if (dist_X > 0)
			return MOVE_R;
		else
			return MOVE_L;
	}
	
	if (robot->orientation != robot->dest_ori)
		return ROTATE;
	
	return robot->dest.value == CELL_BOX ? BOX_GRAB : BOX_DROP;
}
*/

int CalcNextMove2(struct _robot* robot) //running in circles
{
	if (robot->charging)
	{
		if (robot->battery < robot->capacity)
		{
			robot->battery += CHARGE_CHUNK;
			return NOP;
		}
		
		else //fully charged
		{
			if 		(robot->x == 0)  //left charger
			{
				if (robot->orientation == HOR)
				{
					robot->capacity -= CAPACITY_CHUNK;
					robot->battery  = robot->capacity;
					robot->charging = FALSE;
					AssignDest(robot, CELL_BOX);
					return MOVE_R;
				}
				else
					return ROTATE;
			}	
				
			else if (robot->x == 8)	//right charger
			{
				if (robot->orientation == VER)
				{
					robot->capacity -= CAPACITY_CHUNK;
					robot->battery  = BATTERY_CAPACITY;
					robot->charging = FALSE;
					AssignDest(robot, CELL_BOX);
					return MOVE_D;
				}
				else
					return ROTATE;
			}	
		}
	}
	
	if (robot->dest.x == robot->x && robot->dest.y == robot->y)
	{
		if (robot->orientation == robot->dest_ori)
		{
			switch(robot->dest.value)
			{
				case CELL_BOX:
					return BOX_GRAB;
				case CELL_CONTAINER:
					return BOX_DROP;
				case CELL_CHARGER:
					robot->charging = TRUE;
					return NOP;
			}
		}
		
		else
			return ROTATE;
	}
	
	if (robot->y == 0 && robot->x != 8)
		if (robot->x == 7 && robot->dest.value != CELL_CHARGER)
			return robot->orientation == VER ? MOVE_D : ROTATE;
		else
			return robot->orientation == HOR ? MOVE_R : ROTATE;
		
	if (robot->y == 7 && robot->x != 0)
		return robot->orientation == HOR ? MOVE_L : ROTATE;
	
	if (robot->x == 0 && robot->y != 0)
		if (robot->y == 1 && robot->dest.value != CELL_CHARGER)
			return robot->orientation == HOR ? MOVE_R : ROTATE;
		else
			return robot->orientation == VER ? MOVE_U : ROTATE;
	
	if (robot->x == 8 && robot->y != 7)
		return robot->orientation == VER ? MOVE_D : ROTATE;
	
	if (robot->x == 1 && robot->y == 1)
		return robot->orientation == VER ? MOVE_U : ROTATE;
	
	if (robot->x == 7 && robot->y == 1)
		return robot->orientation == HOR ? MOVE_R : ROTATE;
	
	return NOP;
}

void GetPair(struct _robot* robot)
{	
	if (pairs.cur == pairs.length)
	{
		pairs.eof = TRUE;
		printf("GetPair(): End of LOG file...\n");
		return;
	}
	robot->pair[0] = pairs.data[pairs.cur][0];
	robot->pair[1] = pairs.data[pairs.cur][1];
	++pairs.cur;
}