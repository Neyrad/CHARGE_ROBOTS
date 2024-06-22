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
        
				if (ValidNeighbor(robot, TempNodeToFind.X, TempNodeToFind.Y, TempNodeToFind.T))
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
/*
bool IsLastFreeCell(int X, int Y, int T)
{
	int freeCells = 0;
	if (ValidNeighbor(X  , Y  , T+1))
		++freeCells;
	if (ValidNeighbor(X+1, Y  , T+1))
		++freeCells;
	if (ValidNeighbor(X-1, Y  , T+1))
		++freeCells;
	if (ValidNeighbor(X  , Y+1, T+1))
		++freeCells;
	if (ValidNeighbor(X  , Y-1, T+1))
		++freeCells;
	assert(freeCells > 0);
	return freeCells == 1;
}

bool BlockingSomeonesWay(int X, int Y, int T, int AgentNumber)
{
	return InBounds      (X, Y, T) 
		&& isReserved    (X, Y, T) 
		&& CustomGetMap  (X, Y, T) != AgentNumber // not our reservation
		&& IsLastFreeCell(X, Y, T);
}
*/
float GetGScore(AStar_Node* Current, AStar_Node* Neighbor) // Anti-deadlock
{
	int aboveCurrent  = CustomGetMap(Current->X,  Current->Y,  Neighbor->T);
	int belowNeighbor = CustomGetMap(Neighbor->X, Neighbor->Y, Current->T);
	
	bool aboveCurrentAlreadyReserved  = aboveCurrent != CELL_EMPTY  && aboveCurrent != CELL_IN &&
									    aboveCurrent != CELL_OUT    && aboveCurrent != CELL_CHARGER;
									   
	bool belowNeighborAlreadyReserved = belowNeighbor != CELL_EMPTY && belowNeighbor != CELL_IN &&
									    belowNeighbor != CELL_OUT   && belowNeighbor != CELL_CHARGER;
	
	if (aboveCurrentAlreadyReserved && belowNeighborAlreadyReserved) // Dead-lock
		return BIG_NUMBER;
	else
	{
		int AgentNumber = CustomGetMap(Current->X, Current->Y, Current->T);
		/*
		if (	BlockingSomeonesWay(Neighbor->X,     Neighbor->Y,     Current->T, AgentNumber)
			||  BlockingSomeonesWay(Neighbor->X + 1, Neighbor->Y,     Current->T, AgentNumber)
			||  BlockingSomeonesWay(Neighbor->X - 1, Neighbor->Y,     Current->T, AgentNumber)
			||  BlockingSomeonesWay(Neighbor->X, 	 Neighbor->Y + 1, Current->T, AgentNumber)
			||  BlockingSomeonesWay(Neighbor->X, 	 Neighbor->Y - 1, Current->T, AgentNumber))
			return BIG_NUMBER;
		else*/
			return 1;
	}
}

bool InBounds(int X, int Y, int T)
{
	return X >= 0 && X < warehouse.size_x
		&& Y >= 0 && Y < warehouse.size_y
		&& T >= 0 && T < SIZE;
}

bool ValidNeighbor(struct _robot* robot, int X, int Y, int T)
{
	if (!InBounds(X, Y, T))
		return false;
	//assert(InBounds(X, Y, T));
	
	bool OurCellCharger = robot->goal_cell == CELL_CHARGER && robot->destination.x == X && robot->destination.y == Y;
	if (ChargerIsBusy[Y][X] && !OurCellCharger)
	{
		//printf("\n\n\n\n\n\n\n\n\n(%d, %d) ---> INVALID\n\n\n\n\n\n\n\n\n\n\n\n", X, Y);
		return false;
	}
	
	int Cell = CustomGetMap(X, Y, T);
	return Cell == CELL_EMPTY || Cell == CELL_IN 
		|| Cell == CELL_OUT   || Cell == CELL_CHARGER;
}