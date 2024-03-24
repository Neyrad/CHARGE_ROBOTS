#include "model.h"

void AssignDest(struct _robot* robot, CELL goal_cell)
{	
	if      (goal_cell == CELL_IN)
		GetPair(robot);
		
	//else if (goal == CELL_CHARGER)
		//choose the closest free charger
	
	robot->goal_cell = goal_cell;
}

void EmergencyMapFill(struct _robot* robot) // DynamicMap? alt name
{
	struct square goal = robot->goal_cell == CELL_IN? ins.elem[robot->in_num]: outs.elem[robot->out_num];		 
	assert(Valid(goal));
	
	EmergencyMapInit(&robot->emergency_map);
	robot->emergency_map.elem[goal.y][goal.x] = 0;

	EmergencyFill(robot, goal);
}

bool isFlower(struct square center)
{
	struct square left  = {center.x - 1, center.y	 };
	struct square right = {center.x + 1, center.y	 };
	struct square up    = {center.x	   , center.y - 1};
	struct square down  = {center.x	   , center.y + 1};
	
	if (warehouse.room[center.y][center.x] == CELL_IN)
	{
		if (   warehouse.robots[center.y][center.x] != CELL_EMPTY && \
			(!Valid(left)  || warehouse.robots[left.y][left.x]   == CELL_EMPT_ROBOT_HOR || warehouse.room[left.y][left.x]   == CELL_WALL) && \
			(!Valid(right) || warehouse.robots[right.y][right.x] == CELL_EMPT_ROBOT_HOR || warehouse.room[right.y][right.x] == CELL_WALL) && \
			(!Valid(up)    || warehouse.robots[up.y][up.x]       == CELL_EMPT_ROBOT_VER || warehouse.room[up.y][up.x]       == CELL_WALL) && \
			(!Valid(down)  || warehouse.robots[down.y][down.x]   == CELL_EMPT_ROBOT_VER || warehouse.room[down.y][down.x]   == CELL_WALL))
			return true;
		else
			return false;
	}
	
	if (warehouse.room[center.y][center.x] == CELL_OUT)
	{
		if (   warehouse.robots[center.y][center.x] != CELL_EMPTY && \
			(!Valid(left)  || warehouse.robots[left.y][left.x]   == CELL_FULL_ROBOT_HOR || warehouse.room[left.y][left.x]   == CELL_WALL) && \
			(!Valid(right) || warehouse.robots[right.y][right.x] == CELL_FULL_ROBOT_HOR || warehouse.room[right.y][right.x] == CELL_WALL) && \
			(!Valid(up)    || warehouse.robots[up.y][up.x]       == CELL_FULL_ROBOT_VER || warehouse.room[up.y][up.x]       == CELL_WALL) && \
			(!Valid(down)  || warehouse.robots[down.y][down.x]   == CELL_FULL_ROBOT_VER || warehouse.room[down.y][down.x]   == CELL_WALL))
			return true;
		else
			return false;
	}
	
	printf("isFlower(): ERROR! Unknown goal cell...\n");
	return false;
}

bool FlowerFormation(struct _robot* robot)
{
	struct square left  = {robot->x - 1, robot->y	 };
	struct square right = {robot->x + 1, robot->y	 };
	struct square up    = {robot->x	   , robot->y - 1};
	struct square down  = {robot->x	   , robot->y + 1};
	
	if (Valid(left)  && warehouse.room[left.y][left.x]   == robot->goal_cell)
		return isFlower(left);
	
	if (Valid(right) && warehouse.room[right.y][right.x] == robot->goal_cell)
		return isFlower(right);
	
	if (Valid(up)    && warehouse.room[up.y][up.x]       == robot->goal_cell)
		return isFlower(up);
		
	if (Valid(down)  && warehouse.room[down.y][down.x]   == robot->goal_cell)
		return isFlower(down);
		
	return false;		
}

int CalcNextMove(struct _robot* robot)
{	
	if (robot->time_in_action > 1) //current action not finished
	{
		robot->time_in_action -= 1;
		return NOP;
	}

	if (robot->escape_flower && robot->x == robot->flower_goal.x && robot->y == robot->flower_goal.y)
	{
		robot->escape_flower = false;
		robot->emergency = false;
	}
/*
	if (robot->stuck > MOVES_STUCK_LIMIT * 400)
	{
		DumpRobots();
		assert(false);
	}
*/
	if (robot->stuck > MOVES_STUCK_LIMIT)
	{
		if (NoOneInEmergencyMode())
		{
			robot->stuck = 0;
			robot->emergency = true;
			
			if (FlowerFormation(robot))
			{
				struct square goal = RandomValidSquare();	 
				assert(Valid(goal));
				robot->escape_flower = true;
				robot->flower_goal = goal;
				EmergencyMapInit(&robot->emergency_map);
				robot->emergency_map.elem[goal.y][goal.x] = 0;
				EmergencyFill(robot, goal);
			}
			
			else
			{
				EmergencyMapFill(robot); //new dynamic map until reaching the destination
									//if stuck again we get here again and recreate the dynamic map
			
				if (robot->emergency_map.elem[robot->y][robot->x] == BIG_NUMBER) // no way out
					robot->emergency = false;
			}
		}
		else
		{
			robot->escape_flower = false; //stuck again 0_o
			robot->emergency = false;
		}
	}

	struct map* Map = robot->emergency? 		    &robot->emergency_map:  \
					  robot->goal_cell == CELL_IN?  &in_maps[robot->in_num]:
					  robot->goal_cell == CELL_OUT? &out_maps[robot->out_num]: NULL;
	assert(Map);

	switch (robot->goal_cell)
	{
		case CELL_IN:
			if (robot->x == ins.elem[robot->in_num].x && \
				robot->y == ins.elem[robot->in_num].y)
				return robot->cur_ori == robot->dest_ori? LOAD: ROTATE;
			break;
			
		case CELL_OUT:
			if (robot->x == outs.elem[robot->out_num].x && \
				robot->y == outs.elem[robot->out_num].y)
				return robot->cur_ori == robot->dest_ori? UNLOAD: ROTATE;
			break;
			
		case CELL_CHARGER:
			robot->battery.charging = true;
			//TODO
			return NOP;
	}
	
	struct square cur   = {robot->x	   , robot->y	 };
				
	struct square left  = {robot->x - 1, robot->y	 };
	struct square right = {robot->x + 1, robot->y	 };
	struct square up    = {robot->x	   , robot->y - 1};
	struct square down  = {robot->x	   , robot->y + 1};
				
				
	int value 	= Map->elem[cur.y][cur.x];
				
	int value_l = Valid(left)?  Map->elem[left.y][left.x]:   BIG_NUMBER;
	int value_r = Valid(right)? Map->elem[right.y][right.x]: BIG_NUMBER;
	int value_u = Valid(up)? 	Map->elem[up.y][up.x]:   	 BIG_NUMBER;
	int value_d = Valid(down)?  Map->elem[down.y][down.x]:   BIG_NUMBER;
				
	//printf("robot->in_num = %d, robot->out_num = %d\n", robot->in_num, robot->out_num);
	//printf("value = %d, value_l = %d, value_r = %d, value_u = %d, value_d = %d\n", \
			value,	    value_l, 	  value_r, 	    value_u, 	  value_d);
				
	if (value_l < value)
		return robot->cur_ori == HOR? MOVE_L: ROTATE;
				
	if (value_r < value)
		return robot->cur_ori == HOR? MOVE_R: ROTATE;
				
	if (value_u < value)
		return robot->cur_ori == VER? MOVE_U: ROTATE;
	
	if (value_d < value)
		return robot->cur_ori == VER? MOVE_D: ROTATE;
	
	robot->stuck += 1;
	
	return NOP;
}


void EmergencyFill(struct _robot* robot, struct square cur)
{
	struct map* map = &robot->emergency_map;
	
	int level = map->elem[cur.y][cur.x];
	
	if (!Valid(cur))
		return;
	
	if (map->covered[cur.y][cur.x])
		return;
	
	struct square left  = {cur.x - 1, cur.y	   };
	struct square right = {cur.x + 1, cur.y	   };
	struct square up    = {cur.x	, cur.y - 1};
	struct square down  = {cur.x	, cur.y + 1};
	
	
	if (ValidEmptyExcludingCurRobot(robot, left)  && map->elem[left.y][left.x]   > level + 1)
	{
		map->elem   [left.y][left.x]   = level + 1;
		map->covered[left.y][left.x]   = false;
	}
	
	if (ValidEmptyExcludingCurRobot(robot, right) && map->elem[right.y][right.x] > level + 1)
	{
		map->elem   [right.y][right.x] = level + 1;
		map->covered[right.y][right.x] = false;
	}
	
	if (ValidEmptyExcludingCurRobot(robot, up) 	  && map->elem[up.y][up.x]       > level + 1)
	{
		map->elem   [up.y][up.x] 	   = level + 1;
		map->covered[up.y][up.x]	   = false;
	}
	
	if (ValidEmptyExcludingCurRobot(robot, down)  && map->elem[down.y][down.x]   > level + 1)
	{
		map->elem   [down.y][down.x]   = level + 1;
		map->covered[down.y][down.x]   = false;
	}
	
	map->covered[cur.y][cur.x] = true;
	
	EmergencyFill(robot, left);
	EmergencyFill(robot, right);
	EmergencyFill(robot, up);
	EmergencyFill(robot, down);
}


void Fill(struct map* map, struct square cur)
{	
	int level = map->elem[cur.y][cur.x];
	
	if (!Valid(cur))
		return;
	
	if (map->covered[cur.y][cur.x])
		return;
	
	struct square left  = {cur.x - 1, cur.y	   };
	struct square right = {cur.x + 1, cur.y	   };
	struct square up    = {cur.x	, cur.y - 1};
	struct square down  = {cur.x	, cur.y + 1};
	
	
	if (Valid(left)  && map->elem[left.y][left.x]   > level + 1)
	{
		map->elem   [left.y][left.x]   = level + 1;
		map->covered[left.y][left.x]   = false;
	}
	
	if (Valid(right) && map->elem[right.y][right.x] > level + 1)
	{
		map->elem   [right.y][right.x] = level + 1;
		map->covered[right.y][right.x] = false;
	}
	
	if (Valid(up) 	 && map->elem[up.y][up.x]       > level + 1)
	{
		map->elem   [up.y][up.x] 	   = level + 1;
		map->covered[up.y][up.x]	   = false;
	}
	
	if (Valid(down)  && map->elem[down.y][down.x]   > level + 1)
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