#include "model.h"

bool EveryoneResponded(int* arr, int N)
{
	int cnt = 0;
	for (int i = 0; i < N; ++i)
		if(arr[i])
			++cnt;
	return cnt == N;
}

bool NoOneInEmergencyMode()
{
	for (int i = 0; i < Robots.N; ++i)
		if (Robots.elem[i].emergency)
			return false;
	return true;
}

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

void ValidateMacros()
{
	assert(MAX_CYCLES_LiFePO4    <= MAX_CYCLES_OF_ALL_TYPES);
	assert(MAX_CYCLES_LiNiMnCoO2 <= MAX_CYCLES_OF_ALL_TYPES);
	assert(MAX_CYCLES_LeadAcid   <= MAX_CYCLES_OF_ALL_TYPES);
	assert(MAX_CYCLES_LiCoO2     <= MAX_CYCLES_OF_ALL_TYPES);
}

bool Valid(struct square A)
{
	return A.x >= 0 && A.x < warehouse.size_x && A.y >= 0 && A.y < warehouse.size_y \
		   && warehouse.room[A.y][A.x] != CELL_WALL;
}

bool ValidEmpty(struct square A)
{	
	return Valid(A) && (warehouse.robots[A.y][A.x] == CELL_EMPTY);
}

bool ValidEmptyExcludingCurRobot(struct _robot* robot, struct square A)
{
	// considering the square occupied by our robot as an empty square
	
	return Valid(A) && A.x == robot->x && A.y == robot->y || ValidEmpty(A);
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