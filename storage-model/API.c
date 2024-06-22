#include "model.h"

extern int GVT;

AStarNode_List* AllNodesGSet;

int CustomGetMap(int x, int y, int t)
{
	if (x < 0 || x >= warehouse.size_x || y < 0 || y >= warehouse.size_y)
		return CELL_WALL;
	
	assert(t < SIZE);
	return ReservationTable.items[(ReservationTable.front + t) % SIZE].elem[y][x];
}

int GetTrueMap(int x, int y)
{
	if (x < 0 || x >= warehouse.size_x || y < 0 || y >= warehouse.size_y)
		return CELL_WALL;
	
	return ReservationTable.map.elem[y][x];
}

float CostOfGoal(int X1, int Y1, int X2, int Y2)
{
	return DistanceBetween(X1, Y1, X2, Y2);
}

int EmptySpaceLayer(int X, int Y, int T)
{	
	for (int CurrentLayer = T; CurrentLayer < SIZE - 1; ++CurrentLayer)
		if (!isReserved(X, Y, CurrentLayer) && !isReserved(X, Y, CurrentLayer + 1))
			return CurrentLayer;
	return SIZE - 2;
}

void AStar_GetRoute(struct _robot* robot, square Start, square End, int AgentNumber, int waitUntil)
{
	int StartX = Start.x;
	int StartY = Start.y;
	int StartT = robot->time_layer;
	
	int EndX   = End.x;
	int EndY   = End.y;
	
	//printf("Finding Route for ROBOT #%d from (%d,%d) to (%d,%d)...\n", AgentNumber-70, StartX, StartY, EndX, EndY);
	//printf("ROBOT #%d (%d, %d) stack: ", robot->num_in_array + 1, robot->x, robot->y); displayRobotCommands(robot);
	//printf("robot->commands_end = (%d, %d)\n", robot->commands_end.X, robot->commands_end.Y);
	
	if (StartX == EndX && StartY == EndY && waitUntil == 0)
		return;
	
	int NextInSolutionPos;
	NodeDataMap dataMap[MAX_ROOM_SIZE_Y][MAX_ROOM_SIZE_X][SIZE];
	
	for (int t = 0; t < SIZE; ++t)
		for (int y = 0; y < MAX_ROOM_SIZE_Y; ++y)
			for (int x = 0; x < MAX_ROOM_SIZE_X; ++x)
			{
				dataMap[y][x][t].GScore   = 0.0;
				dataMap[y][x][t].FScore   = 0.0;
				dataMap[y][x][t].CameFrom = NULL;
			}

	
	AStar_Node* Solution = AStar_Find(robot, StartX, StartY, StartT, EndX, EndY, dataMap, waitUntil);
  
	if (!Solution)
	{
#if __DEBUG_PRINT
		printf("No solution was found [GVT = %d]\n", GVT);
#endif
		printf("No solution was found [GVT = %d] for agent [%d]\n", GVT, AgentNumber);
		printf("Start = (%d, %d), End = (%d, %d)\n", StartX, StartY, EndX, EndY);
		DumpRobots();
		for (int y = 0; y < warehouse.size_y; ++y)
		{
			for (int x = 0; x < warehouse.size_x; ++x)
				printf("%d ", ChargerIsBusy[y][x]);
			printf("\n");
		}
		assert(false);
/*
		if (StartX == EndX && StartY == EndY)
		{
			printf("No solution was found [GVT = %d]\n", GVT);
			assert(false);
		}
		else
		{
			AStar_GetRoute(robot, Start, Start, AgentNumber, SIZE - 2);
		}
*/
	}
	
	int map_to_display[MAX_ROOM_SIZE_Y][MAX_ROOM_SIZE_X];
  
	for (int y = 0; y < MAX_ROOM_SIZE_Y; ++y)
		for (int x = 0; x < MAX_ROOM_SIZE_X; ++x)
			map_to_display[y][x] = (ReservationTable.map.elem[y][x] == CELL_WALL)? '#': ' ';
  
	AStar_Node* SolutionNavigator = NULL;
	AStar_Node* NextInSolution = Solution;
  
	// NextInSolution will actually refer to the next node from end to start (that is, we're going reverse from the target).
	if (NextInSolution)
	{
#if __DEBUG_PRINT
		printf("Backtracking from the target\n");
#endif
		do
		{
#if __DEBUG_PRINT
			printf("(%d, %d, %d)\n", NextInSolution->X, NextInSolution->Y, NextInSolution->T);
#endif
			NextInSolution->NextInSolvedPath = SolutionNavigator;
			SolutionNavigator = NextInSolution;
			NextInSolution = dataMap[NextInSolution->Y][NextInSolution->X][NextInSolution->T].CameFrom;
		}
		while (SolutionNavigator->X != StartX || SolutionNavigator->Y != StartY || SolutionNavigator->T != StartT);
	}
  
	if (SolutionNavigator) // reserve current cell
	{
		Reserve(SolutionNavigator->X, SolutionNavigator->Y, SolutionNavigator->T, AgentNumber);
	}
  
	while (SolutionNavigator)
	{		
		map_to_display[SolutionNavigator->Y][SolutionNavigator->X] = 'O';
	
		int movX = SolutionNavigator->NextInSolvedPath? ((AStar_Node*)SolutionNavigator->NextInSolvedPath)->X - SolutionNavigator->X: 0;
		int movY = SolutionNavigator->NextInSolvedPath? ((AStar_Node*)SolutionNavigator->NextInSolvedPath)->Y - SolutionNavigator->Y: 0;
		
		message_type tmp;
		if (!movX)
		{
			if (!movY)
				tmp = NOP;
			else
				tmp = movY < 0? MOVE_U: MOVE_D;
		}
		else
			tmp = movX > 0? MOVE_R: MOVE_L;

		assert(tmp);
		
		if (SolutionNavigator->NextInSolvedPath)
		{		
			RQ_enQueue(robot, tmp);
			Reserve(SolutionNavigator->X + movX, SolutionNavigator->Y + movY, SolutionNavigator->T + 1, AgentNumber);
		}

		else //last command
		{	
			robot->commands_end.x = SolutionNavigator->X;
			robot->commands_end.y = SolutionNavigator->Y;
			robot->time_layer	  = SolutionNavigator->T;
		
			if (SolutionNavigator->X == robot->destination.x && SolutionNavigator->Y == robot->destination.y)
			{
				if (!isReserved(robot->destination.x, robot->destination.y, SolutionNavigator->T + 1))
				{
					switch (robot->goal_cell)
					{
						case CELL_IN:
							RQ_enQueue(robot, LOAD);//robot->cur_ori == robot->dest_ori? LOAD: ROTATE;
							break;				
						case CELL_OUT:
							RQ_enQueue(robot, UNLOAD);//robot->cur_ori == robot->dest_ori? UNLOAD: ROTATE;
							break;				
						case CELL_CHARGER:
							RQ_enQueue(robot, NOP);
							break;
					}
				
					Reserve(SolutionNavigator->X, SolutionNavigator->Y, SolutionNavigator->T + 1, AgentNumber);
					robot->time_layer	  = SolutionNavigator->T + 1;
				}
				else
				{
					AStar_GetRoute(robot, End, End, AgentNumber, EmptySpaceLayer(SolutionNavigator->X, SolutionNavigator->Y, SolutionNavigator->T + 1));
				}
			}
		}

		SolutionNavigator = SolutionNavigator->NextInSolvedPath;
		
		//if (robot->num_in_array == 0)
		//	printf("ROBOT #%d timelayer = %d\n", robot->num_in_array + 1, robot->time_layer);
		//displayReservationTableAlt();
	}
	
	map_to_display[StartY][StartX] = '1';
	map_to_display[EndY  ][EndX  ] = '2';
  /*
	for (int y = 0; y < MAX_ROOM_SIZE_Y; ++y)
	{
		for (int x = 0; x < MAX_ROOM_SIZE_X; ++x)
		{
			switch (map_to_display[y][x])
			{
				case '#':
					printf("#");
					break;
				case ' ':
					printf(" ");
					break;
				case '1':
					printf("1");
					break;
				case '2':
					printf("2");
					break;
				default:
					// Assumed path.
					printf("O");
			}
		}	
		printf("\n");
	}
	*/
	RemoveAllFromNodeList(&AllNodesGSet, true);
}