#include "model.h"

bool EveryoneResponded(int* arr, int N)
{
	int cnt = 0;
	for (int i = 0; i < N; ++i)
		if (arr[i])
			++cnt;
	return cnt == N;
}

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
		   && warehouse.room[A.y][A.x] != CELL_WALL;
}

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