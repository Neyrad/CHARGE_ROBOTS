#include "model.h"

void AssignDest(struct _robot* robot, CELL goal_cell)
{	
	square tmp;
	
	if      (goal_cell == CELL_IN)
	{
		GetPair(robot);
		tmp.x = ins.elem[robot->in_num].x;
		tmp.y = ins.elem[robot->in_num].y;
	}
	
	else if (goal_cell == CELL_OUT)
	{
		tmp.x = outs.elem[robot->out_num].x;
		tmp.y = outs.elem[robot->out_num].y;
	}
	
	else if (goal_cell == CELL_CHARGER)
	{
		if(!AssignEmptyChargerIfPossible(robot))
			robot->charger_num = rand() % chargers.size;
		
		tmp.x = chargers.elem[robot->charger_num].x,
		tmp.y = chargers.elem[robot->charger_num].y;
	}
	
	else
		assert(false);
	
	robot->goal_cell = goal_cell;
	robot->destination = tmp;
	AStar_GetRoute(robot, NewSquare(robot->x, robot->y), robot->destination, 70 + robot->num_in_array + 1);
	printf("AssignDest: printing stack and reservation table...\n");
	printf("ROBOT #%d (%d, %d) stack: ", robot->num_in_array + 1, robot->x, robot->y); displayRobotCommands(robot);
	displayReservationTableAlt();
	PrintRoomAndRobots();
}

bool AssignEmptyChargerIfPossible(struct _robot* robot)
{
	for (int i = 0; i < chargers.size; ++i)
		if (warehouse.robots[chargers.elem[i].y][chargers.elem[i].x] == CELL_EMPTY)
		{
			robot->charger_num = i;
			return true;
		}
	return false;
}

int CalcNextMove(struct _robot* robot)
{	
	if (robot->time_in_action > 1) //current action not finished
	{
		robot->time_in_action -= 1;
		return NOP;
	}

	if (robot->battery.charging)
	{
		if (robot->battery.charge < robot->battery.capacity)
		{
			robot->battery.charge += CHARGE_CHUNK;
			if (robot->battery.charge > robot->battery.capacity)
				robot->battery.charge = robot->battery.capacity;
			robot->battery.time_spent_charging += 1;
		}
		
		else //fully charged
		{
			robot->battery.times_recharged += 1;
			robot->battery.capacity 		= CalculateCapacity(robot);
			robot->battery.charge  			= robot->battery.capacity;
			robot->battery.charging 		= false;
			AssignDest(robot, CELL_IN);
		}
		
		return NOP;
	}

/*	if (robot->x == robot->destination.x && robot->y == robot->destination.y)
	{
		assert(RQ_isEmpty(robot));
		switch (robot->goal_cell)
		{
			case CELL_IN:
				return LOAD;//robot->cur_ori == robot->dest_ori? LOAD: ROTATE;
			
			case CELL_OUT:
				return UNLOAD;//robot->cur_ori == robot->dest_ori? UNLOAD: ROTATE;
			
			case CELL_CHARGER:
				robot->battery.charging = true;
				return NOP;
		}
	}
*/
	if (robot->commands.n_elems < SIZE / 2 && (robot->commands_end.x != robot->destination.x  || \
											   robot->commands_end.y != robot->destination.y) || RQ_isEmpty(robot))
	{
		if (RQ_isEmpty(robot))
		{
			robot->commands_end.x = robot->x;
			robot->commands_end.y = robot->y;
		}
		AStar_GetRoute(robot, robot->commands_end, robot->destination, 70 + robot->num_in_array + 1);
		//printf("CalcNextMove: printing stack and reservation table...\n");
		//printf("ROBOT #%d (%d, %d) stack: ", robot->num_in_array + 1, robot->x, robot->y); displayRobotCommands(robot);
		displayReservationTableAlt();
		//PrintRoomAndRobots();
		assert(!RQ_isEmpty(robot));
	}
	
	//printf("ROBOT #%d (%d, %d) stack: ", robot->num_in_array + 1, robot->x, robot->y); displayRobotCommands(robot);
	//PrintRoomAndRobots();
	return RQ_deQueue(robot);
}

void Fill(struct _map* map, square cur)
{	
	int level = map->elem[cur.y][cur.x];
	
	if (!ValidAlt(cur) && level != 0)
		return;
	
	if (map->covered[cur.y][cur.x])
		return;
	
	square left  = {cur.x - 1, cur.y	};
	square right = {cur.x + 1, cur.y	};
	square up    = {cur.x	 , cur.y - 1};
	square down  = {cur.x	 , cur.y + 1};
	
	
	if (ValidAlt(left)  && map->elem[left.y][left.x]   > level + 1)
	{
		map->elem   [left.y][left.x]   = level + 1;
		map->covered[left.y][left.x]   = false;
	}
	
	if (ValidAlt(right) && map->elem[right.y][right.x] > level + 1)
	{
		map->elem   [right.y][right.x] = level + 1;
		map->covered[right.y][right.x] = false;
	}
	
	if (ValidAlt(up) 	 && map->elem[up.y][up.x]       > level + 1)
	{
		map->elem   [up.y][up.x] 	   = level + 1;
		map->covered[up.y][up.x]	   = false;
	}
	
	if (ValidAlt(down)  && map->elem[down.y][down.x]   > level + 1)
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