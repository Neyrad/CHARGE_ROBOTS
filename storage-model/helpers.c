#include "model.h"

bool EveryoneResponded(int* arr, int N)
{
	int cnt = 0;
	for (int i = 0; i < N; ++i)
		if (arr[i])
			++cnt;
	return cnt == N;
}
/*
bool BlockedFromAllSides(int x, int y)
{
	square left  = {x - 1, y	  };
	square right = {x + 1, y	  };
	square up    = {x	, y - 1};
	square down  = {x	, y + 1};
	
	if (ValidEmpty(left))  return false;
	if (ValidEmpty(right)) return false;
	if (ValidEmpty(up))    return false;
	if (ValidEmpty(down))  return false;

	return true;
}
*/
void ValidateMacros()
{
	assert(MAX_CYCLES_LiFePO4    <= MAX_CYCLES_OF_ALL_TYPES);
	assert(MAX_CYCLES_LiNiMnCoO2 <= MAX_CYCLES_OF_ALL_TYPES);
	assert(MAX_CYCLES_LeadAcid   <= MAX_CYCLES_OF_ALL_TYPES);
	assert(MAX_CYCLES_LiCoO2     <= MAX_CYCLES_OF_ALL_TYPES);
}

bool Valid(square A)
{
	return A.x >= 0 && A.x < warehouse.size_x && A.y >= 0 && A.y < warehouse.size_y \
		   && warehouse.room[A.y][A.x] != CELL_WALL;
}

bool ValidAlt(square A)
{
	return A.x >= 0 && A.x < warehouse.size_x && A.y >= 0 && A.y < warehouse.size_y \
		   && warehouse.room[A.y][A.x] == CELL_EMPTY;
}

/*
bool ValidEmpty(square A)
{	
	return Valid(A) && (warehouse.robots[A.y][A.x] == CELL_EMPTY);
}

bool ValidEmptyExcludingCurRobot(struct _robot* robot, square A)
{
	// considering the square occupied by our robot as an empty square
	
	return Valid(A) && A.x == robot->x && A.y == robot->y || ValidEmpty(A);
}

square RandomValidSquare()
{
	int tries = 100;
	for (int i = 0; i < tries; ++i)
	{
		//int rand_x = (rand() % (warehouse.size_x / 3)) + (warehouse.size_x / 3);
		square random_square = {rand() % warehouse.size_x, rand() % warehouse.size_y};
		if (Valid(random_square) && !BlockedFromAllSides(random_square.x, random_square.y))
			return random_square;
	}
	printf("RandomValidSquare(): ERROR! Unable to find...\n");
	square error = {-1, -1};
	return error;
}
*/
CELL GetNewCellRobot(struct _robot* this)
{
	if (this->loaded)
	{
		if (this->cur_ori == VER)
			return CELL_FULL_ROBOT_VER;
		else
			return CELL_FULL_ROBOT_HOR;
	}
	
	else
	{
		if (this->cur_ori == VER)
			return CELL_EMPT_ROBOT_VER;
		else
			return CELL_EMPT_ROBOT_HOR;
	}
}

square NewSquare(int x, int y)
{
	square tmp;
	tmp.x = x;
	tmp.y = y;
	return tmp;
}