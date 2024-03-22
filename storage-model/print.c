#include "model.h"

extern const char* path_to_log_folder;

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
		PrintMapConsole(Robots.elem[i].emergency_map.elem, i);
		//PrintMapConsole(Robots.elem[i].emergency_map.covered, i);
		printf("\n\n\n");
	}
}