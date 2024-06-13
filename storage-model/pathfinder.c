#include "model.h"

float DistanceBetween(int X1, int Y1, int X2, int Y2)
{
	return sqrt((float)((X2-X1) * (X2-X1) + (Y2-Y1) * (Y2-Y1)));
}

void AddToNodeList(AStarNode_List** List, AStar_Node* NodeToAdd, int* LengthPtr)
{
	AStarNode_List* newNode = malloc(sizeof(AStarNode_List));
	newNode->node = NodeToAdd;
	newNode->next = *List;
  
	*List = newNode;
  
	if (LengthPtr)
		(*LengthPtr)++;
}

AStar_Node* CreateNode(int X, int Y, int T, AStarNode_List** AllNodesSet)
{
	AStar_Node* ThisNode = malloc(sizeof(AStar_Node));
	ThisNode->X          = X;
	ThisNode->Y          = Y;
	ThisNode->T          = T;
	ThisNode->Neighbors[0]  = NULL;
	ThisNode->Neighbors[1]  = NULL;
	ThisNode->Neighbors[2]  = NULL;
	ThisNode->Neighbors[3]  = NULL;
	ThisNode->Neighbors[4]  = NULL;
  
	AddToNodeList(AllNodesSet, ThisNode, NULL);
  
	return ThisNode;
}

AStarNode_List* FindInNodeList(AStarNode_List* List, AStar_Node* NodeToFind)
{
	AStarNode_List* FoundNode   = NULL;
	AStarNode_List* CurrentNode = List;
	while (CurrentNode)
	{
		if (CurrentNode->node->X == NodeToFind->X && \
			CurrentNode->node->Y == NodeToFind->Y && \
			CurrentNode->node->T == NodeToFind->T)
		{
			// Found it.
			FoundNode = CurrentNode;
			break;
		}
    
		CurrentNode = CurrentNode->next;
	}
  
	return FoundNode;
}

void RemoveFromNodeList(AStarNode_List** List, AStar_Node* NodeToRemove, int* LengthPtr)
{
	AStarNode_List* CurrentNode   = *List;
	AStarNode_List* PreviousNode  = NULL;
	while (CurrentNode)
	{
		if (CurrentNode->node->X == NodeToRemove->X && \
			CurrentNode->node->Y == NodeToRemove->Y && \
			CurrentNode->node->T == NodeToRemove->T)
		{
			// Found it.
			if (PreviousNode)
				PreviousNode->next = CurrentNode->next;
			else
				*List = CurrentNode->next;
      
			if (LengthPtr)
				(*LengthPtr)--;
			break;
		}
		else
		{
			PreviousNode = CurrentNode;
			CurrentNode  = CurrentNode->next;
		}
	}
  
	return;
}

void RemoveAllFromNodeList(AStarNode_List** List, bool FreeNodes)
{
	if (!List)
		return;
  
	AStarNode_List* CurrentNode = *List;
	AStarNode_List* NextNode;
  
	while (CurrentNode)
	{
		if (FreeNodes && CurrentNode->node)
			free(CurrentNode->node);
		
		NextNode = CurrentNode->next;
		free(CurrentNode);
		CurrentNode = NextNode;
	}
	*List = NULL;
}

AStar_Node* AStar_Find(struct _robot* robot, int StartX, int StartY, int StartT, int EndX, int EndY, NodeDataMap dataMap[MAX_ROOM_SIZE_Y][MAX_ROOM_SIZE_X][SIZE], int waitUntil)
{
	struct _map* Heuristic = robot->goal_cell == CELL_IN?      &in_maps[robot->in_num]:   \
							 robot->goal_cell == CELL_OUT?     &out_maps[robot->out_num]: \
							 robot->goal_cell == CELL_CHARGER? &charger_maps[robot->charger_num]: NULL;
	assert(Heuristic);
	
	
	AStar_Node*		Neighbor       	 = NULL;
	AStarNode_List* OpenSet        	 = NULL;
	AStarNode_List* ClosedSet      	 = NULL;
	AStarNode_List* NextInOpenSet    = NULL;
	AStar_Node*	    AStar_SolvedPath = NULL;
	int             OpenSetLength    = 0;
	float         	TentativeGScore  = 0;
	float         	TentativeFScore  = 0;
	float        	LowestFScore;
	float        	NextFScore;
	AStar_Node     	TempNodeToFind;
	AStarNode_List*	TempNeighborNode;
	int 			neighborPosInLists;
  
	AllNodesGSet = NULL;
  
	if ((GetTrueMap(StartX, StartY) == CELL_WALL) || (GetTrueMap(EndX, EndY) == CELL_WALL))
	{
#if __DEBUG_PRINT
		printf("Impossible. Either the start or end point is in a wall\n");
#endif
		return NULL;
	}
  
	AStar_Node* Current = CreateNode(StartX, StartY, StartT, &AllNodesGSet);
  
	dataMap[Current->Y][Current->X][Current->T].GScore   = 0.0;
	dataMap[Current->Y][Current->X][Current->T].FScore   = dataMap[Current->Y][Current->X][Current->T].GScore + Heuristic->elem[Current->Y][Current->X];
	dataMap[Current->Y][Current->X][Current->T].CameFrom = NULL;
  
	AddToNodeList(&OpenSet, Current, &OpenSetLength);
  
	while (OpenSetLength)
	{
		Current = NULL;
		NextInOpenSet = OpenSet;
		LowestFScore = dataMap[NextInOpenSet->node->Y][NextInOpenSet->node->X][NextInOpenSet->node->T].FScore;
#if __DEBUG_PRINT
		printf("######## START:  Finding lowest one.  Starting with: %d, %d, %d\n", NextInOpenSet->node->X, NextInOpenSet->node->Y, NextInOpenSet->node->T);
#endif
		while (NextInOpenSet)
		{
			NextFScore = dataMap[NextInOpenSet->node->Y][NextInOpenSet->node->X][NextInOpenSet->node->T].FScore;
			if (!Current || LowestFScore > NextFScore)
			{
				Current = NextInOpenSet->node;
				LowestFScore = NextFScore;
			}
			NextInOpenSet = NextInOpenSet->next;
		}
    
#if __DEBUG_PRINT
		printf("Current: %d, %d, %d\n", Current->X, Current->Y, Current->T);
#endif
	
		if (Current->X == EndX && Current->Y == EndY && Current->T >= waitUntil || Current->T >= SIZE - 2) // leaving space for LOAD/UNLOAD/CHARGE
		{
			robot->commands_end.x = Current->X;
			robot->commands_end.y = Current->Y;
			// We reached the goal.
#if __DEBUG_PRINT
			printf("Goal achieved! Current->T = %d\n", Current->T);
#endif
			AStar_SolvedPath = Current;
			break;
		}
    
#if __DEBUG_PRINT
		printf("Removing current from open set\n");
#endif
		RemoveFromNodeList(&OpenSet, Current, &OpenSetLength);
#if __DEBUG_PRINT
		printf("Adding current to closed set\n");
#endif
		if (!FindInNodeList(ClosedSet, Current))
			AddToNodeList(&ClosedSet, Current, NULL);
#if __DEBUG_PRINT
		printf("Analyzing neighbors\n");
#endif
		for (int neighbor_pos = 0; neighbor_pos < 5; ++neighbor_pos)
		{
			Neighbor = Current->Neighbors[neighbor_pos];
			if (!Neighbor)
			{
				TempNodeToFind.X = Current->X;
				TempNodeToFind.Y = Current->Y;
				TempNodeToFind.T = Current->T + 1;
				switch(neighbor_pos)
				{
					case 0:
						TempNodeToFind.Y--;
						break;
					case 1:
						TempNodeToFind.X++;
						break;
					case 2:
						TempNodeToFind.Y++;
						break;
					case 3:
						TempNodeToFind.X--;
						break;
					default:
						// Assumed pause
						break;
				}
        
				if (ValidNeighbor(&TempNodeToFind, EndX, EndY))
				{
					TempNeighborNode = FindInNodeList(AllNodesGSet, &TempNodeToFind);
					if (TempNeighborNode)
					{
#if __DEBUG_PRINT
						printf("Selecting already existing neighbor\n");
#endif
						Neighbor = TempNeighborNode->node;
					}
					else
					{
#if __DEBUG_PRINT
						printf("Creating new neighbor\n");
#endif
						Neighbor = CreateNode(TempNodeToFind.X, TempNodeToFind.Y, TempNodeToFind.T, &AllNodesGSet);
					}
          
#if __DEBUG_PRINT
					printf("Linking current node and neighbor as, well, neighbors\n");
#endif
					Current->Neighbors[neighbor_pos] = Neighbor;
				}
				else
				{
#if __DEBUG_PRINT
					printf("Node is an obstacle or not within map.  Skipping\n");
#endif
				}
			}
      
			if (Neighbor)
			{
				TentativeGScore = dataMap[Current->Y][Current->X][Current->T].GScore + GetGScore(Current, Neighbor)/*DistanceBetween(Current->X, Current->Y, Neighbor->X, Neighbor->Y)*/;
#if __DEBUG_PRINT
				printf("Tentative G score: %f\n", TentativeGScore);
#endif
				TentativeFScore = TentativeGScore + Heuristic->elem[Neighbor->Y][Neighbor->X];
#if __DEBUG_PRINT
				printf("Tentative F score: %f\n", TentativeFScore);
#endif
        
				if (!FindInNodeList(ClosedSet, Neighbor) || TentativeFScore < dataMap[Neighbor->Y][Neighbor->X][Neighbor->T].FScore)
				{
					if (!FindInNodeList(OpenSet, Neighbor) || TentativeFScore < dataMap[Neighbor->Y][Neighbor->X][Neighbor->T].FScore)
					{
						if (!dataMap[Neighbor->Y][Neighbor->X][Neighbor->T].CameFrom)
							dataMap[Neighbor->Y][Neighbor->X][Neighbor->T].CameFrom = Current;
						
						dataMap[Neighbor->Y][Neighbor->X][Neighbor->T].GScore   = TentativeGScore;
						dataMap[Neighbor->Y][Neighbor->X][Neighbor->T].FScore   = TentativeFScore;
          
						if (!FindInNodeList(OpenSet, Neighbor))
						{
#if __DEBUG_PRINT
							printf("Added neighbor to open set\n");
#endif
							AddToNodeList(&OpenSet, Neighbor, &OpenSetLength);
						}
					}
					else
					{
#if __DEBUG_PRINT
						printf("Neighbor already in open set and tentative F score more than neighbor's F score.  Not adding to open set\n");
#endif
					}
				}
				else
				{
#if __DEBUG_PRINT
					printf("Neighbor in closed set and tentative F score is more than or equal to neighbor's F score.  Not adding to open set\n");
#endif
				}
			}
		}
	}
  
	RemoveAllFromNodeList(&OpenSet,   0);
	RemoveAllFromNodeList(&ClosedSet, 0);
  
	return AStar_SolvedPath;
}

float GetGScore(AStar_Node* Current, AStar_Node* Neighbor) // Anti-deadlock
{
	CELL aboveCurrent  = ReservationTable.items[(ReservationTable.front + Neighbor->T) % SIZE].elem[Current->Y][Current->X];
	CELL belowNeighbor = ReservationTable.items[(ReservationTable.front + Current->T)  % SIZE].elem[Neighbor->Y][Neighbor->X];
	
	bool aboveCurrentAlreadyReserved  = aboveCurrent != CELL_EMPTY  && aboveCurrent != CELL_IN &&
									    aboveCurrent != CELL_OUT    && aboveCurrent != CELL_CHARGER;
									   
	bool belowNeighborAlreadyReserved = belowNeighbor != CELL_EMPTY && belowNeighbor != CELL_IN &&
									    belowNeighbor != CELL_OUT   && belowNeighbor != CELL_CHARGER;
	
	if (aboveCurrentAlreadyReserved && belowNeighborAlreadyReserved) // Dead-lock
		return BIG_NUMBER;
	else
		return 1;
}

bool ValidNeighbor(AStar_Node* TempNodeToFind, int EndX, int EndY)
{
	bool in_bounds = TempNodeToFind->X >= 0 && TempNodeToFind->X < warehouse.size_x && TempNodeToFind->Y >= 0 && TempNodeToFind->Y < warehouse.size_y;
	//bool is_end_point = TempNodeToFind->X == EndX && TempNodeToFind->Y == EndY;
	bool empty_cell   = CustomGetMap(TempNodeToFind->X, TempNodeToFind->Y, TempNodeToFind->T) == CELL_EMPTY;
	bool in_cell      = CustomGetMap(TempNodeToFind->X, TempNodeToFind->Y, TempNodeToFind->T) == CELL_IN;
	bool out_cell     = CustomGetMap(TempNodeToFind->X, TempNodeToFind->Y, TempNodeToFind->T) == CELL_OUT;
	bool charger_cell = CustomGetMap(TempNodeToFind->X, TempNodeToFind->Y, TempNodeToFind->T) == CELL_CHARGER;
	
	//return in_bounds && (empty_cell || is_end_point && (in_cell || out_cell || charger_cell));
	return in_bounds && (empty_cell || in_cell || out_cell || charger_cell);
}