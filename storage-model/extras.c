#include "model.h"

//The C extras file for a ROSS model
//This file includes:
// - all functions that are not from ROSS

#define MAX_PATH_TO_LOG_FOLDER 256

extern int glb_time;

extern const char* path_to_log_folder;
extern const char* path_to_room_file;
extern const char* path_to_robots_file;
extern const char* path_to_pairs;

void PrintMovesInit() // empties the log file beforehand
{
	char path[MAX_PATH_TO_LOG_FOLDER];
    sprintf(path, "%s/robots.log", path_to_log_folder);
	LogFile = fopen(path, "w"); //erasing contents from the previous launch
	fclose(LogFile);
	LogFile = fopen(path, "a");
}

void PrintMoves()
{
	for (int i = 0; i < Robots.N; ++i)
		fprintf(LogFile, "%c", CurMove[i]);
	fprintf(LogFile, "\n");
}

void PrintPairs()
{
	for (int y = 0; y < pairs.length; ++y)
	{
        for (int x = 0; x < 2; ++x)
		{
            printf("%d", pairs.elem[y][x]);
            if (x == 0)
				printf(","); //no comma at the end of a line
        }
        printf("\n");
    }
}

//Converts a CSV file to a 2D int array
void Parse(const char* path, int arr[MAX_ROOM_SIZE_Y][MAX_ROOM_SIZE_X])
{
    struct stat stats;
    stat(path, &stats);
    FILE* f = fopen(path, "r");
    assert(f); //to check correctness of the path
    char buf[MAX_ROOM_SIZE_Y * (MAX_ROOM_SIZE_X + 1) * 2 + MAX_COMMENT_LENGTH]; // * 2 for commas, +1 for \n\r
    fread(buf, sizeof(buf[0]), stats.st_size, f);
    fclose(f);

    int start_index = 0;
    if (buf[0] == '#')
	{
        for (start_index = 0; buf[start_index] != '\n' ; ++start_index)
            ;
        ++start_index; //next symbol after #bla-bla-bla\n
    }
    
    warehouse.size_y = 0;
    int x = 0;
    for (int i = start_index; i < stats.st_size; ++i)
	{
        if (buf[i] == '\n')
		{
            ++warehouse.size_y;
            warehouse.size_x = x;
            x = 0;
            continue;
        }
        if (isdigit(buf[i]))
		{
            if ((i + 1 < stats.st_size) && isdigit(buf[i+1]))
			{  //processing 2-digit numbers
                arr[warehouse.size_y][x] = (buf[i] - '0') * 10 + (buf[i+1] - '0');
                ++i;
            }
            else
                arr[warehouse.size_y][x] = buf[i] - '0';
            ++x;
        }
    }
}

void PairsInit(const char* path)
{
	pairs.elem = calloc(MAX_INPUT_LENGTH, sizeof(int*));
	for (size_t i = 0; i < MAX_INPUT_LENGTH; ++i)
		pairs.elem[i] = calloc(2, sizeof(int));

    struct stat stats;
    stat(path, &stats);
    FILE* f = fopen(path, "r");
    assert(f); //to check correctness of the path
	assert(stats.st_size < MAX_INPUT_LENGTH * 4);
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
            pairs.elem[pairs.length][x] = buf[i] - '0';
            ++x;
        }
    }
	pairs.cur = 0;
	pairs.eof = false;
}

void FreePairs()
{	
	for (size_t i = 0; i < MAX_INPUT_LENGTH; ++i)
		free(pairs.elem[i]);
	free(pairs.elem);
}

void RobotsInit()
{
    Robots.N = 0;
    for (int y = 0; y < warehouse.size_y; ++y)
        for (int x = 0; x < warehouse.size_x; ++x)
            if (warehouse.robots[y][x] == CELL_EMPT_ROBOT_VER   		|| \
                warehouse.robots[y][x] == CELL_EMPT_ROBOT_HOR   		|| \
				warehouse.robots[y][x] == CELL_FULL_ROBOT_VER || \
                warehouse.robots[y][x] == CELL_FULL_ROBOT_HOR) 
			{
				if (warehouse.robots[y][x] == CELL_EMPT_ROBOT_VER || \
					warehouse.robots[y][x] == CELL_EMPT_ROBOT_HOR)
					Robots.elem[Robots.N].loaded = false;
				else
					Robots.elem[Robots.N].loaded = true;
				
                Robots.elem[Robots.N].x = x;
                Robots.elem[Robots.N].y = y;
                
                if (warehouse.robots[y][x] == CELL_EMPT_ROBOT_VER || \
				    warehouse.robots[y][x] == CELL_FULL_ROBOT_VER)
				    Robots.elem[Robots.N].cur_ori = VER;
				else
					Robots.elem[Robots.N].cur_ori = HOR;

                ++Robots.N;
                assert(Robots.N <= MAX_ROBOTS);
            }
	assert(Robots.N);
}

void RobotsPrint()
{
	char path[MAX_PATH_TO_LOG_FOLDER];
    sprintf(path, "%s/robot_numbers.csv", path_to_log_folder);
	FILE* f = fopen(path, "w");
	
    for (int i = 0; i < Robots.N; ++i)
	{
		struct _robot cur = Robots.elem[i];
		fprintf(f, "%d,%d,%d,%d\n", cur.x, cur.y, cur.cur_ori, cur.loaded);
        printf("Robot #%d is located at (%d, %d) ", \
                      i+1, Robots.elem[i].x, Robots.elem[i].y);
        if (Robots.elem[i].cur_ori == VER)
			printf("VER\n");
		else
			printf("HOR\n");
    }
	
	fclose(f);
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

bool EveryoneResponded(int* arr, int N)
{
	int cnt = 0;
	for (int i = 0; i < N; ++i)
		if(arr[i])
			++cnt;
	return cnt == N;
}

void AssignDest(struct _robot* robot, CELL goal_cell)
{	
	if      (goal_cell == CELL_IN)
		GetPair(robot);
		
	//else if (goal == CELL_CHARGER)
		//choose the closest free charger
	
	robot->goal_cell = goal_cell;
}

void EmergencyMapFill(struct _robot* robot) // DynamicMap? alt name
{
	//printf("Emergency\n");
	struct square goal = robot->goal_cell == CELL_IN? ins.elem[robot->in_num]: outs.elem[robot->out_num];		 
	assert(Valid(goal));
	
	EmergencyMapInit(&robot->emergency_map);
	robot->emergency_map.elem[goal.y][goal.x] = 0;
	/*
	PrintMapConsole(robot->emergency_map.elem, 444);
	PrintMapConsole(robot->emergency_map.covered, 555);
	
	printf("------------------------------1--------------------------------\n");


	for (int y = 0; y < warehouse.size_y; ++y)
		for (int x = 0; x < warehouse.size_x; ++x)
		{
			struct square cur = {x, y};
			if (!ValidEmpty(cur))
				printf("ValidEmpty(%d, %d) == false\n", cur.x, cur.y);
		}
	
	printf("------------------------------1--------------------------------\n");
	*/
	EmergencyFill(robot, goal);
	/*
	printf("------------------------------2--------------------------------\n");


	for (int y = 0; y < warehouse.size_y; ++y)
		for (int x = 0; x < warehouse.size_x; ++x)
		{
			struct square cur = {x, y};
			if (!ValidEmpty(cur))
				printf("ValidEmpty(%d, %d) == false\n", cur.x, cur.y);
		}
	
	printf("------------------------------2--------------------------------\n");
	
	
	
	PrintMapConsole(robot->emergency_map.elem, 666);
	PrintMapConsole(robot->emergency_map.covered, 777);
	
	*/
}

bool NoOneInEmergencyMode()
{
	for (int i = 0; i < Robots.N; ++i)
		if (Robots.elem[i].emergency)
			return false;
	return true;
}

void DumpRobots()
{
	for (int i = 0; i < Robots.N; ++i)
	{
		printf("Robots.elem[%d].x                 = %d\n", i, Robots.elem[i].x);
		printf("Robots.elem[%d].y                 = %d\n", i, Robots.elem[i].y);
		printf("Robots.elem[%d].stuck             = %d\n", i, Robots.elem[i].stuck);
		printf("Robots.elem[%d].emergency         = %d\n", i, Robots.elem[i].emergency);
		printf("Robots.elem[%d].escape_flower     = %d\n", i, Robots.elem[i].escape_flower);
		printf("Robots.elem[%d].flower_goal       = (%d, %d)\n", i, Robots.elem[i].flower_goal.x, Robots.elem[i].flower_goal.y);
		printf("Robots.elem[%d].loaded            = %d\n", i, Robots.elem[i].loaded);
		printf("Robots.elem[%d].in_num            = %d\n", i, Robots.elem[i].in_num);
		printf("Robots.elem[%d].out_num           = %d\n", i, Robots.elem[i].out_num);
		//printf("NoOneInEmergencyMode() && !BlockedFromAllSides() = %d\n", NoOneInEmergencyMode() && !BlockedFromAllSides(&Robots.elem[i]));
		
		
		
		
		PrintMapConsole(Robots.elem[i].emergency_map.elem, i);
		//PrintMapConsole(Robots.elem[i].emergency_map.covered, i);
		printf("\n\n\n");
	}
}

int n_prints = 0;

bool BlockedFromAllSides(int x, int y)
{
	struct square left  = {x - 1, y	  };
	struct square right = {x + 1, y	  };
	struct square up    = {x	, y - 1};
	struct square down  = {x	, y + 1};
	
	if (ValidEmpty(left))  return false;
	if (ValidEmpty(right)) return false;
	if (ValidEmpty(up))    return false;
	if (ValidEmpty(down))  return false;

	return true;
}

struct square RandomValidSquare()
{
	int tries = 100;
	for (int i = 0; i < tries; ++i)
	{
		struct square random_square = {rand() % warehouse.size_x, rand() % warehouse.size_y};
		if (Valid(random_square) && !BlockedFromAllSides(random_square.x, random_square.y))
			return random_square;
	}
	printf("RandomValidSquare(): ERROR! Unable to find...\n");
	struct square error = {-1, -1};
	return error;
}

bool isFlower(struct square center)
{
	struct square left  = {center.x - 1, center.y	 };
	struct square right = {center.x + 1, center.y	 };
	struct square up    = {center.x	   , center.y - 1};
	struct square down  = {center.x	   , center.y + 1};
	
	if (warehouse.room[center.y][center.x] == CELL_IN)
	{
		if (   warehouse.robots[center.y][center.x] == CELL_FULL_ROBOT_VER || warehouse.robots[center.y][center.x] == CELL_FULL_ROBOT_HOR && \
			(!Valid(left)  || warehouse.robots[left.y][left.x]   == CELL_EMPT_ROBOT_HOR || warehouse.room[left.y][left.x]   == CELL_WALL) && \
			(!Valid(right) || warehouse.robots[right.y][right.x] == CELL_EMPT_ROBOT_HOR || warehouse.room[right.y][right.x] == CELL_WALL) && \
			(!Valid(up)    || warehouse.robots[up.y][up.x]       == CELL_EMPT_ROBOT_VER || warehouse.room[up.y][up.x]       == CELL_WALL) && \
			(!Valid(down)  || warehouse.robots[down.y][down.x]   == CELL_EMPT_ROBOT_VER || warehouse.room[down.y][down.x]   == CELL_WALL))
			return true;
		else
			return false;
	}
	
	if (warehouse.room[center.y][center.x] == CELL_OUT)
	{
		if (   warehouse.robots[center.y][center.x] == CELL_EMPT_ROBOT_VER || warehouse.robots[center.y][center.x] == CELL_EMPT_ROBOT_HOR && \
			(!Valid(left)  || warehouse.robots[left.y][left.x]   == CELL_FULL_ROBOT_HOR || warehouse.room[left.y][left.x]   == CELL_WALL) && \
			(!Valid(right) || warehouse.robots[right.y][right.x] == CELL_FULL_ROBOT_HOR || warehouse.room[right.y][right.x] == CELL_WALL) && \
			(!Valid(up)    || warehouse.robots[up.y][up.x]       == CELL_FULL_ROBOT_VER || warehouse.room[up.y][up.x]       == CELL_WALL) && \
			(!Valid(down)  || warehouse.robots[down.y][down.x]   == CELL_FULL_ROBOT_VER || warehouse.room[down.y][down.x]   == CELL_WALL))
			return true;
		else
			return false;
	}
	
	printf("isFlower(): ERROR! Unknown goal cell...\n");
	return false;
}

bool FlowerFormation(struct _robot* robot)
{
	struct square left  = {robot->x - 1, robot->y	 };
	struct square right = {robot->x + 1, robot->y	 };
	struct square up    = {robot->x	   , robot->y - 1};
	struct square down  = {robot->x	   , robot->y + 1};
	
	struct square flower_center = {-1, -1};
	
	if (Valid(left)  && warehouse.room[left.y][left.x]   == robot->goal_cell)
		return isFlower(left);
	
	if (Valid(right) && warehouse.room[right.y][right.x] == robot->goal_cell)
		return isFlower(right);
	
	if (Valid(up)    && warehouse.room[up.y][up.x]       == robot->goal_cell)
		return isFlower(up);
		
	if (Valid(down)  && warehouse.room[down.y][down.x]   == robot->goal_cell)
		return isFlower(down);
		
	return false;		
}

int CalcNextMove(struct _robot* robot)
{	
	if (robot->time_in_action > 1) //current action not finished
	{
		robot->time_in_action -= 1;
		return NOP;
	}
/*
	if (n_prints > 2)
		assert(false);

	if (robot->stuck > (MOVES_STUCK_LIMIT * 350))
	{
		DumpRobots();
		++n_prints;
	}
*/
	if (robot->escape_flower && robot->x == robot->flower_goal.x && robot->y == robot->flower_goal.y)
	{
		robot->escape_flower = false;
		robot->emergency = false;
	}

	if (robot->stuck > MOVES_STUCK_LIMIT)
	{
		if (NoOneInEmergencyMode()/* && !BlockedFromAllSides(robot->x, robot->y)*/)
		{
			robot->stuck = 0;
			robot->emergency = true;
			
			if (FlowerFormation(robot))
			{
				struct square goal = RandomValidSquare();	 
				assert(Valid(goal));
				robot->escape_flower = true;
				robot->flower_goal = goal;
				EmergencyMapInit(&robot->emergency_map);
				robot->emergency_map.elem[goal.y][goal.x] = 0;
				EmergencyFill(robot, goal);
				//PrintMapConsole(robot->emergency_map.elem, 123);
			}
			
			else
			{
				EmergencyMapFill(robot); //new dynamic map until reaching the destination
									//if stuck again we get here again and recreate the dynamic map
			
				if (robot->emergency_map.elem[robot->y][robot->x] == BIG_NUMBER) // no way out
					robot->emergency = false;
			}
		}
		else
		{
			robot->escape_flower = false; //stuck again 0_o
			robot->emergency = false;
		}
	}

	struct map* Map = robot->emergency? 		    &robot->emergency_map:  \
					  robot->goal_cell == CELL_IN?  &in_maps[robot->in_num]:
					  robot->goal_cell == CELL_OUT? &out_maps[robot->out_num]: NULL;
	assert(Map);

	switch (robot->goal_cell)
	{
		case CELL_IN:
			if (robot->x == ins.elem[robot->in_num].x && \
				robot->y == ins.elem[robot->in_num].y)
				return robot->cur_ori == robot->dest_ori? LOAD: ROTATE;
			break;
			
		case CELL_OUT:
			if (robot->x == outs.elem[robot->out_num].x && \
				robot->y == outs.elem[robot->out_num].y)
				return robot->cur_ori == robot->dest_ori? UNLOAD: ROTATE;
			break;
			
		case CELL_CHARGER:
			robot->battery.charging = true;
			//TODO
			return NOP;
	}
	
	struct square cur   = {robot->x	   , robot->y	 };
				
	struct square left  = {robot->x - 1, robot->y	 };
	struct square right = {robot->x + 1, robot->y	 };
	struct square up    = {robot->x	   , robot->y - 1};
	struct square down  = {robot->x	   , robot->y + 1};
				
				
	int value 	= Map->elem[cur.y][cur.x];
				
	int value_l = Valid(left)?  Map->elem[left.y][left.x]:   BIG_NUMBER;
	int value_r = Valid(right)? Map->elem[right.y][right.x]: BIG_NUMBER;
	int value_u = Valid(up)? 	Map->elem[up.y][up.x]:   	 BIG_NUMBER;
	int value_d = Valid(down)?  Map->elem[down.y][down.x]:   BIG_NUMBER;
				
	//printf("robot->in_num = %d, robot->out_num = %d\n", robot->in_num, robot->out_num);
	//printf("value = %d, value_l = %d, value_r = %d, value_u = %d, value_d = %d\n", \
			value,	    value_l, 	  value_r, 	    value_u, 	  value_d);
				
	if (value_l < value)
		return robot->cur_ori == HOR? MOVE_L: ROTATE;
				
	if (value_r < value)
		return robot->cur_ori == HOR? MOVE_R: ROTATE;
				
	if (value_u < value)
		return robot->cur_ori == VER? MOVE_U: ROTATE;
	
	if (value_d < value)
		return robot->cur_ori == VER? MOVE_D: ROTATE;
	
	return NOP;
}

void GetPair(struct _robot* robot)
{	
	if (pairs.cur == pairs.length)
	{
		pairs.eof = true;
		printf("GetPair(): End of input pairs file...\n");
		return;
	}
	robot->in_num  = pairs.elem[pairs.cur][0];
	robot->out_num = pairs.elem[pairs.cur][1];
	++pairs.cur;
}

void InitBDM(struct _robot* robot, BatteryType BT)
{
	robot->battery.BDM_cur = 0;
	robot->battery.type = BT;
	switch(robot->battery.type)
	{
		case LiFePO4:
		{
			const float A =  0.00000005;
			const float B = -0.000287;
			const float C =  1;
	
			for (int i = 0; i < MAX_CYCLES_LiFePO4; ++i)
				robot->battery.BDM[i] = (A*i*i + B*i + C) * BATTERY_CAPACITY;
		}
			break;
		case LiNiMnCoO2:
		{
			const float A1 =  -0.00022;
			const float B1 =   1;
			const float A2 =  -0.001;
			const float B2 =   1.663;
			const int FUNC_CHANGE_CYCLE = 850;	
	
			for (int i = 0;   i < FUNC_CHANGE_CYCLE && i < MAX_CYCLES_LiNiMnCoO2; ++i)
				robot->battery.BDM[i] = (A1*i + B1) * BATTERY_CAPACITY;
			for (int i = FUNC_CHANGE_CYCLE;            i < MAX_CYCLES_LiNiMnCoO2; ++i)
				robot->battery.BDM[i] = (A2*i + B2) * BATTERY_CAPACITY;
		}
			break;
		case LeadAcid:
		{
			/*
			
			y = k*x + b
			
			k = (y2 - y1) / (x2 - x1)
			b = y1 - k*x1
			
			*/
	
			struct point points[10] = {{0,   1},    {25,  0.99}, {50,  0.92}, \
									   {75,  0.85}, {100, 0.77}, {125, 0.75}, \
									   {150, 0.72}, {175, 0.65}, {200, 0.62}, \
									   {225, 0.6}};
									   
			float k = 0;
			float b = 0;
			int cur_pnt = 0;
	
			for (int i = 0; i < MAX_CYCLES_LeadAcid; ++i)
			{
				if (i == points[cur_pnt].x && cur_pnt < 9)
				{
					k = (points[cur_pnt + 1].y - points[cur_pnt].y) / \
						(points[cur_pnt + 1].x - points[cur_pnt].x);
						
					b = points[cur_pnt].y - k * points[cur_pnt].x;
					
					//printf("InitBDM(): k = %f, b = %f\n", k, b);
					
					++cur_pnt;
				}
				robot->battery.BDM[i] = (k * i + b) * BATTERY_CAPACITY;
			}	
		}
			break;
		case LiCoO2:
		{
			/*
			
			y = k*x + b
			
			k = (y2 - y1) / (x2 - x1)
			b = y1 - k*x1
			
			*/
	
			struct point points[12] = {{0,   1},    {45,  1.003}, {90,  1},     \
									   {430,  0.9}, {450, 0.89},  {500, 0.87},  \
									   {535, 0.85}, {580, 0.81},  {600, 0.765}, \
									   {625, 0.71}, {650, 0.655}, {680, 0.62}    };
									   
			float k = 0;
			float b = 0;
			int cur_pnt = 0;
	
			for (int i = 0; i < MAX_CYCLES_LiCoO2; ++i)
			{
				if (i == points[cur_pnt].x && cur_pnt < 11)
				{
					k = (points[cur_pnt + 1].y - points[cur_pnt].y) / \
						(points[cur_pnt + 1].x - points[cur_pnt].x);
						
					b = points[cur_pnt].y - k * points[cur_pnt].x;
					
					//printf("InitBDM(): k = %f, b = %f\n", k, b);
					
					++cur_pnt;
				}
				robot->battery.BDM[i] = (k * i + b) * BATTERY_CAPACITY;
			}	
		}
			break;
		default:
			printf("InitBDM(): Unknown battery type [%d]\n", robot->battery.type);
			return;
	}
}

void PrintBDM(struct _robot* robot, const char* print_to)
{
	int MAX_CYCLES = -1;
	switch(robot->battery.type)
	{
		case LiFePO4:
			MAX_CYCLES = MAX_CYCLES_LiFePO4;
			printf("PrintBDM(): Battery type LiFePO4\n");
			break;
		case LiNiMnCoO2:
			MAX_CYCLES = MAX_CYCLES_LiNiMnCoO2;
			printf("PrintBDM(): Battery type LiNiMnCoO2\n");
			break;
		case LeadAcid:
			MAX_CYCLES = MAX_CYCLES_LeadAcid;
			printf("PrintBDM(): Battery type LeadAcid\n");
			break;
		case LiCoO2:
			MAX_CYCLES = MAX_CYCLES_LiCoO2;
			printf("PrintBDM(): Battery type LiCoO2\n");
			break;
		default:
			printf("PrintBDM(): Unknown battery type [%d]\n", robot->battery.type);
			return;
	}
	
	if (print_to == "console" || print_to == "Console" || print_to == "CONSOLE")
	{
		for (int i = 0; i < MAX_CYCLES; ++i)
			printf("robot->battery.BDM[%d] = %d\n", i, robot->battery.BDM[i]);
	}
	
	else
	{
		FILE* f = fopen(print_to, "w");
		assert(f);
		
		for (int i = 0; i < MAX_CYCLES; ++i)
			fprintf(f, "%d,%f\n", i, (float)robot->battery.BDM[i] / (float)BATTERY_CAPACITY);
		
		fclose(f);
	}
}

int CalculateCapacity(struct _robot* robot)
{
	int MAX_CYCLES = -1;
	switch(robot->battery.type)
	{
		case LiFePO4:
			MAX_CYCLES = MAX_CYCLES_LiFePO4;
			break;
		case LiNiMnCoO2:
			MAX_CYCLES = MAX_CYCLES_LiNiMnCoO2;
			break;
		case LeadAcid:
			MAX_CYCLES = MAX_CYCLES_LeadAcid;
			break;
		case LiCoO2:
			MAX_CYCLES = MAX_CYCLES_LiCoO2;
			break;
		default:
			printf("CalculateCapacity(): Unknown battery type [%d]\n", robot->battery.type);
			return -1;
	}
	
	if (robot->battery.BDM_cur == MAX_CYCLES - 1)
	{
		//BatteryDeath();
		printf("\n\n\nCalculateCapacity(): battery of the robot #??? died...\n\n\n\n");
	}
	//assert(robot->battery.BDM_cur < MAX_CYCLES);
	//assert(robot->battery.BDM[robot->battery.BDM_cur] > 0);
	return robot->battery.BDM[robot->battery.BDM_cur++];
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

void ValidateMacros()
{
	assert(MAX_CYCLES_LiFePO4    <= MAX_CYCLES_OF_ALL_TYPES);
	assert(MAX_CYCLES_LiNiMnCoO2 <= MAX_CYCLES_OF_ALL_TYPES);
	assert(MAX_CYCLES_LeadAcid   <= MAX_CYCLES_OF_ALL_TYPES);
	assert(MAX_CYCLES_LiCoO2     <= MAX_CYCLES_OF_ALL_TYPES);
}

void FilesInit()
{
	Parse(path_to_room_file,   warehouse.room);
	Parse(path_to_robots_file, warehouse.robots);
	PrintMovesInit();
}

void InitROSS()
{
	FilesInit();
	PairsInit(path_to_pairs);
    RobotsInit();
	
	FindInsOuts();
	InitMaps();
	for (int i = 0; i < ins.size; ++i)
	{
		Fill(&in_maps[i], ins.elem[i]);
		PrintMapConsole(in_maps[i].elem, i);
	}
	for (int i = 0; i < outs.size; ++i)
	{
		Fill(&out_maps[i], outs.elem[i]);
		PrintMapConsole(out_maps[i].elem, i);
	}
	
	RobotsPrint();
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

bool ValidEmptyExcludingCurRobot(struct _robot* robot, struct square A)
{
	// considering the square occupied by our robot as an empty square
	
	return Valid(A) && A.x == robot->x && A.y == robot->y || ValidEmpty(A);
}

void EmergencyFill(struct _robot* robot, struct square cur)
{
	struct map* map = &robot->emergency_map;
	
	//PrintMapConsole(map->elem, 228);
	
	int level = map->elem[cur.y][cur.x];
	
	if (!Valid(cur))
		return;
	
	if (map->covered[cur.y][cur.x])
		return;
	
	//PrintMapConsole(map->elem, 228);
	
	struct square left  = {cur.x - 1, cur.y	   };
	struct square right = {cur.x + 1, cur.y	   };
	struct square up    = {cur.x	, cur.y - 1};
	struct square down  = {cur.x	, cur.y + 1};
	
	
	if (ValidEmptyExcludingCurRobot(robot, left)  && map->elem[left.y][left.x]   > level + 1)
	{
		map->elem   [left.y][left.x]   = level + 1;
		map->covered[left.y][left.x]   = false;
	}
	
	if (ValidEmptyExcludingCurRobot(robot, right) && map->elem[right.y][right.x] > level + 1)
	{
		map->elem   [right.y][right.x] = level + 1;
		map->covered[right.y][right.x] = false;
	}
	
	if (ValidEmptyExcludingCurRobot(robot, up) 	  && map->elem[up.y][up.x]       > level + 1)
	{
		map->elem   [up.y][up.x] 	   = level + 1;
		map->covered[up.y][up.x]	   = false;
	}
	
	if (ValidEmptyExcludingCurRobot(robot, down)  && map->elem[down.y][down.x]   > level + 1)
	{
		map->elem   [down.y][down.x]   = level + 1;
		map->covered[down.y][down.x]   = false;
	}
	
	map->covered[cur.y][cur.x] = true;
	//printf("EmFill(): covered\n");
	
	EmergencyFill(robot, left);
	EmergencyFill(robot, right);
	EmergencyFill(robot, up);
	EmergencyFill(robot, down);
}


void Fill(struct map* map, struct square cur)
{	
	int level = map->elem[cur.y][cur.x];
	
	if (!Valid(cur))
		return;
	
	if (map->covered[cur.y][cur.x])
		return;
	
	struct square left  = {cur.x - 1, cur.y	   };
	struct square right = {cur.x + 1, cur.y	   };
	struct square up    = {cur.x	, cur.y - 1};
	struct square down  = {cur.x	, cur.y + 1};
	
	
	if (Valid(left)  && map->elem[left.y][left.x]   > level + 1)
	{
		map->elem   [left.y][left.x]   = level + 1;
		map->covered[left.y][left.x]   = false;
	}
	
	if (Valid(right) && map->elem[right.y][right.x] > level + 1)
	{
		map->elem   [right.y][right.x] = level + 1;
		map->covered[right.y][right.x] = false;
	}
	
	if (Valid(up) 	 && map->elem[up.y][up.x]       > level + 1)
	{
		map->elem   [up.y][up.x] 	   = level + 1;
		map->covered[up.y][up.x]	   = false;
	}
	
	if (Valid(down)  && map->elem[down.y][down.x]   > level + 1)
	{
		map->elem   [down.y][down.x]   = level + 1;
		map->covered[down.y][down.x]   = false;
	}
	
	map->covered[cur.y][cur.x] = true;
	
	Fill(map, left);
	Fill(map, right);
	Fill(map, up);
	Fill(map, down);
}

bool Valid(struct square A)
{
	//printf("Valid(%d, %d) = %d\n", A.x, A.y, A.x >= 0 && A.x < warehouse.size_x && A.y >= 0 && A.y < warehouse.size_y \
		   && warehouse.room[A.y][A.x] != CELL_WALL);
	
	return A.x >= 0 && A.x < warehouse.size_x && A.y >= 0 && A.y < warehouse.size_y \
		   && warehouse.room[A.y][A.x] != CELL_WALL;
}

bool ValidEmpty(struct square A)
{
	//printf("ValidEmpty(%d, %d)", A.x, A.y);
/*	
	for (int i = 0; i < Robots.N; ++i)
	{
		//printf("cur (%d, %d) ROBOT #%d\n", Robots.elem[i].x, Robots.elem[i].y, i);
		if (Robots.elem[i].x == A.x && Robots.elem[i].y == A.y)
		{
			//printf("occupied (%d, %d) ROBOT #%d\n", A.x, A.y, i);
			return false;
		}
	}
	//printf(" = true, calling Valid(%d, %d)...\n", A.x, A.y);
*/

	//PrintMapConsole(warehouse.robots, 123);
	//printf("%d\n", Valid(A) && (warehouse.robots[A.y][A.x] == CELL_EMPTY));
	
	return Valid(A) && (warehouse.robots[A.y][A.x] == CELL_EMPTY);
}

void InitMaps()
{
	for (int i = 0; i < N_INS; ++i)
	{
		for (int y = 0; y < MAX_ROOM_SIZE_Y; ++y)
			for (int x = 0; x < MAX_ROOM_SIZE_X; ++x)
			{
				in_maps[i].elem   [y][x] = BIG_NUMBER;
				in_maps[i].covered[y][x] = false;
			}
		in_maps[i].elem[ins.elem[i].y][ins.elem[i].x] = 0; // IN
	}
	
	for (int i = 0; i < N_OUTS; ++i)
	{
		for (int y = 0; y < MAX_ROOM_SIZE_Y; ++y)
			for (int x = 0; x < MAX_ROOM_SIZE_X; ++x)
			{
				out_maps[i].elem   [y][x] = BIG_NUMBER;
				out_maps[i].covered[y][x] = false;
			}
		out_maps[i].elem[outs.elem[i].y][outs.elem[i].x] = 0; // OUT
	}
}

void EmergencyMapInit(struct map* emergency_map)
{
	for (int y = 0; y < MAX_ROOM_SIZE_Y; ++y)
		for (int x = 0; x < MAX_ROOM_SIZE_X; ++x)
		{
			emergency_map->elem   [y][x] = BIG_NUMBER;
			emergency_map->covered[y][x] = false;
		}
}

void FindInsOuts()
{
	ins.size = 0;
	for (int y = 0; y < warehouse.size_y; ++y)
		for (int x = 0; x < warehouse.size_x; ++x)
			if (warehouse.room[y][x] == CELL_IN)
			{
				struct square tmp = {x, y};
				ins.elem[ins.size] = tmp;
				++ins.size;
			}
			
	outs.size = 0;
	for (int y = 0; y < warehouse.size_y; ++y)
		for (int x = 0; x < warehouse.size_x; ++x)
			if (warehouse.room[y][x] == CELL_OUT)
			{
				struct square tmp = {x, y};
				outs.elem[outs.size] = tmp;
				++outs.size;
			}
}

void PrintMapConsole(int map[MAX_ROOM_SIZE_Y][MAX_ROOM_SIZE_X], int Num)
{
	printf("\n*** WEIGHT MAP FOR IN/OUT NUMBER %d ***\n", Num);
	for (int y = 0; y < warehouse.size_y; ++y)
	{
		for (int x = 0; x < warehouse.size_x; ++x)
			printf("%3d ", map[y][x]);
		printf("\n");
	}
}

void PrintCovered(bool cov[MAX_ROOM_SIZE_Y][MAX_ROOM_SIZE_X], int Num)
{
	printf("\n*** COVERAGE MAP FOR OUT NUMBER %d ***\n", Num);
	for (int y = 0; y < warehouse.size_y; ++y)
	{
		for (int x = 0; x < warehouse.size_x; ++x)
			printf("%d ", cov[y][x]);
		printf("\n");
	}
}

void PrintRoom()
{
	printf("\n***        ROOM GEOMETRY        ***\n");
	for (int y = 0; y < warehouse.size_y; ++y)
	{
		for (int x = 0; x < warehouse.size_x; ++x)
			printf("%d ", warehouse.room[y][x]);
		printf("\n");
	}
}