#include "model.h"

int GVT = 0;

bool RT_isFull()
{
	return (ReservationTable.front == ReservationTable.rear + 1) || (ReservationTable.front == 0 && ReservationTable.rear == SIZE - 1);
}

bool RT_isEmpty()
{
	return ReservationTable.front == -1;
}

void InitReservationTable(int map[MAX_ROOM_SIZE_Y][MAX_ROOM_SIZE_X])
{
	for (int y = 0; y < warehouse.size_y; ++y)
		for (int x = 0; x < warehouse.size_x; ++x)
			ReservationTable.map.elem[y][x] = map[y][x];
	
	ReservationTable.front = -1;
	ReservationTable.rear  = -1;
	for (int i = 0; i < SIZE; ++i)
		RT_enQueue(ReservationTable.map);
}

void PrintLayer(layer L, int i)
{
	printf("Layer #%d:\n", i);
	for (int y = 0; y < warehouse.size_y; ++y)
	{
		printf("[ ");
		for (int x = 0; x < warehouse.size_x; ++x)
			printf("%2d ", L.elem[y][x]);
		printf("]\n");
	}
}

void PrintLayerToFile(char* folderPath, layer L, int i)
{
	char path[MAX_PATH_TO_LOG_FOLDER];
    sprintf(path, "%s/%d.csv", folderPath, i);
	FILE* f = fopen(path, "w");
	for (int y = 0; y < warehouse.size_y; ++y)
	{
		for (int x = 0; x < warehouse.size_x; ++x)
		{
			switch (L.elem[y][x])
			{
				case 1:
				case 3:
				case 5:
				case 7:
					fprintf(f, "0");
					break;
				default:
					fprintf(f, "%d", L.elem[y][x]);
					break;
			}
			if (x != warehouse.size_x - 1)
				fprintf(f, ",");
		}
		fprintf(f, "\n");
	}
	fclose(f);
}

layer CreateLayer(int i)
{
	layer L;
	for (int y = 0; y < warehouse.size_y; ++y)
		for (int x = 0; x < warehouse.size_x; ++x)
			L.elem[y][x] = i;
	
	//PrintLayer(L, i);
	return L;
}

void RT_enQueue(layer element)
{
	//printf("GVT = <%d>\n", GVT);
	//for (int i = 0; i < Robots.N; ++i)
	//	printf("Robots.elem[%d].time_layer = <%d>\n", i, Robots.elem[i].time_layer);
	
	++GVT;
	
	if (RT_isFull())
		printf("Queue is full !!!\n");
	else 
	{
		if (ReservationTable.front == -1)
			ReservationTable.front = 0;
		ReservationTable.rear = (ReservationTable.rear + 1) % SIZE;
		ReservationTable.items[ReservationTable.rear] = element;
	}
	
	//displayReservationTableAlt();
}

layer RT_deQueue()
{
	if (RT_isEmpty())
	{
		printf("Queue is empty !!!\n");
		return CreateLayer(-1);
	}
	else
	{
		layer element = ReservationTable.items[ReservationTable.front];
		if (ReservationTable.front == ReservationTable.rear)
		{
			ReservationTable.front = -1;
			ReservationTable.rear = -1;
		} 

		else
			ReservationTable.front = (ReservationTable.front + 1) % SIZE;
		
		return element;
	}
}

void displayReservationTableAlt()
{
	//assert(false);
	
	if (RT_isEmpty())
		printf("Empty Queue\n");
	else
	{
		// to file
		int realNum = 0;
		char path[MAX_PATH_TO_LOG_FOLDER];
		sprintf(path, "/mnt/c/Dev/Base/3dplot/sim/%d", GVT);
		
		struct stat st = {0};
		if (stat(path, &st) == -1)
		{	
			//printf("making new dir...\n");
			mkdir(path, 0700);
			//printf("done!\n");
		}
		
		for (int i = ReservationTable.front; realNum < SIZE; i = (i + 1) % SIZE)
		{
			PrintLayerToFile(path, ReservationTable.items[i], realNum);
			realNum++;
		}
	}
}

void displayReservationTable()
{
	if (RT_isEmpty())
		printf("Empty Queue\n");
	else
	{
		for (int i = ReservationTable.front; i != ReservationTable.rear; i = (i + 1) % SIZE)
			PrintLayer(ReservationTable.items[i], i);
		PrintLayer(ReservationTable.items[ReservationTable.rear], ReservationTable.rear);
	}
}

void Reserve(int X, int Y, int T, int AgentNumber)
{
	assert(X >= 0 && X < warehouse.size_x);
	assert(Y >= 0 && Y < warehouse.size_y);
	assert(T >= 0 && T < SIZE);
	ReservationTable.items[(ReservationTable.front + T) % SIZE].elem[Y][X] = AgentNumber;
}

bool isReserved(int X, int Y, int T)
{
	assert(X >= 0 && X < warehouse.size_x);
	assert(Y >= 0 && Y < warehouse.size_y);
	assert(T >= 0 && T < SIZE);
	CELL Cur = ReservationTable.items[(ReservationTable.front + T) % SIZE].elem[Y][X];
	return Cur != CELL_EMPTY && Cur != CELL_IN && Cur != CELL_OUT && Cur != CELL_CHARGER;
}