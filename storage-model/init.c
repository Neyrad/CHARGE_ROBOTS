#include "model.h"

extern const char* path_to_log_folder;
extern const char* path_to_room_file;
extern const char* path_to_robots_file;
extern const char* path_to_pairs;

void PairsInit(const char* path)
{
	printf("PairsInit(): ALIVE, sizeof(int) = %ld\n", sizeof(int));
	
	
	pairs.elem = calloc(MAX_INPUT_LENGTH, sizeof(int*));	
	for (int i = 0; i < MAX_INPUT_LENGTH; ++i)
		pairs.elem[i] = calloc(2, sizeof(int));

    struct stat stats;
    stat(path, &stats);

    FILE* f = fopen(path, "r");
    assert(f); //to check correctness of the path
	printf("stats.st_size = %ld, MAX_INPUT_LENGTH * 4 = %d\n", stats.st_size, MAX_INPUT_LENGTH * 4);
	assert(stats.st_size <= MAX_INPUT_LENGTH * 4);





 //   char buf[MAX_INPUT_LENGTH * 4]; 
	char* buf = calloc(MAX_INPUT_LENGTH * 4, sizeof(char)); //'1' ',' '2' '\n' -> 4 columns (should I consider \r ???)


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
	
	
	free(buf);
	
	printf("PairsInit(): ALIVE\n");
	//assert(false);
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

void FilesInit()
{
	printf("FilesInit(): ALIVE\n");
	
	Parse(path_to_room_file,   warehouse.room);
	
	printf("FilesInit(): ALIVE\n");
	
	Parse(path_to_robots_file, warehouse.robots);
	
	printf("FilesInit(): ALIVE\n");
	
	PairsInit(path_to_pairs);
	
	printf("FilesInit(): ALIVE\n");
	
	PrintMovesInit();
	
	printf("FilesInit(): ALIVE\n");
}

void InitROSS()
{
	printf("InitROSS(): ALIVE\n");
	
	srand(time(NULL));
	
	printf("InitROSS(): ALIVE\n");
	
	FilesInit();
	
	printf("InitROSS(): ALIVE\n");
	
    RobotsInit();
	
	printf("InitROSS(): ALIVE\n");
	
	FindInsOuts();
	
	printf("InitROSS(): ALIVE\n");
	
	InitMaps();
	
	printf("InitROSS(): ALIVE\n");
	
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
	
	printf("InitROSS(): ALIVE\n");
	
	RobotsPrint();
	
	printf("InitROSS(): ALIVE\n");
}

void PrintMovesInit() // empties the log file beforehand
{
	char path[MAX_PATH_TO_LOG_FOLDER];
    sprintf(path, "%s/robots.log", path_to_log_folder);
	LogFile = fopen(path, "w"); //erasing contents from the previous launch
	fclose(LogFile);
	LogFile = fopen(path, "a");
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