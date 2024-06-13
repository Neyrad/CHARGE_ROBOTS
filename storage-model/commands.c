#include "model.h"

void Rotate(struct _robot* this, int self)
{
	assert(false);
	
	if (this->time_in_action > 1) //busy
		return;
	this->time_in_action = ROTATE_TIME;
					
	if (this->state == MOTION)
		this->battery.charge -= STOP_MOTION_COST;
	this->state = STOP;
					
	if (this->cur_ori == VER)
	{
		this->cur_ori = HOR;
		warehouse.robots_next_step[this->y][this->x] = this->loaded? CELL_FULL_ROBOT_HOR: CELL_EMPT_ROBOT_HOR;
	}
	else
	{
		this->cur_ori = VER;
		warehouse.robots_next_step[this->y][this->x] = this->loaded? CELL_FULL_ROBOT_VER: CELL_EMPT_ROBOT_VER;
	}

	CurMove[self-1] = 'F';
	this->battery.charge -= ROTATE_COST;
}

void Move(struct _robot* this, char direction, int self) // 'U', 'D', 'L', 'R'
{
	if (this->time_in_action > 1) //busy
		return;
	this->time_in_action = MOVE_TIME;
	
	int move_x = 0;
	int move_y = 0;
	direction = toupper(direction);
	switch(direction)
	{
		case 'U':
			//assert(this->cur_ori == VER);
			move_x =  0;
			move_y = -1;
			break;
		case 'D':
			//assert(this->cur_ori == VER);
			move_x =  0;
			move_y = +1;
			break;
		case 'L':
			//assert(this->cur_ori == HOR);
			move_x = -1;
			move_y =  0;
			break;
		case 'R':
			//assert(this->cur_ori == HOR);
			move_x = +1;
			move_y =  0;
			break;
		default:
			assert(false);
			printf("Move(): ERROR!!! Invalid direction\n");
			break;
	}

	square destination = {this->x + move_x, this->y + move_y};
	if (!Valid(destination))
	{
		//printf("destination = (%d, %d)\n", destination.x, destination.y);
		//printf("this->boxes_delivered = %d\n", this->boxes_delivered);
		//displayReservationTable();
		//displayRobotCommands(this);
	}
	assert(Valid(destination));
			
    switch(warehouse.room[destination.y][destination.x])
    {
        case CELL_EMPTY:
		case CELL_IN:
		case CELL_OUT:
		case CELL_CHARGER:
			if (warehouse.robots_next_step[destination.y][destination.x] == CELL_EMPTY)
			{
				warehouse.robots_next_step[destination.y][destination.x] = GetNewCellRobot(this);
				this->x = destination.x;
				this->y = destination.y;
								
				if 		(this->state == MOTION)
					this->battery.charge -= KEEP_MOTION_COST;
				else if (this->state == STOP)
					this->battery.charge -= START_MOTION_COST;
																
				this->state = MOTION;
				CurMove[self-1] = direction;
			}
			else //the way is blocked by another robot
			{
				printf("ROBOT #%d can't go through\n", this->num_in_array + 1);
				
				printf("warehouse.robots_next_step[destination.y][destination.x] == %d\n", warehouse.robots_next_step[destination.y][destination.x]);
				printf("destination = (%d, %d)\n", destination.x, destination.y);
				PrintRoomAndRobots();
				
				displayReservationTableAlt();
				
				for (int i = 0; i < Robots.N; ++i)
					printf("Robots.elem[%d].time_layer = <%d>\n", i, Robots.elem[i].time_layer);
				
				assert(false);
			}
			break;
        case CELL_WALL:
			this->state = STOP;
			assert(false);
			break;
        default:
            break;
    }	
}

void Load(struct _robot* this, int self)
{
	if (this->time_in_action > 1) //busy
		return;
	this->time_in_action = LOAD_TIME;
					
	if (this->state == MOTION)
		this->battery.charge -= STOP_MOTION_COST;
	this->state = STOP;
					
    if (warehouse.room[this->y][this->x] == CELL_IN && this->loaded == false)
    {
        this->loaded = true;
		if (this->cur_ori == VER)
			warehouse.robots_next_step[this->y][this->x] = CELL_FULL_ROBOT_VER;
		else
			warehouse.robots_next_step[this->y][this->x] = CELL_FULL_ROBOT_HOR;
						
		AssignDest(this, CELL_OUT);
		
		CurMove[self-1] = 'I';
    }
}

void Unload(struct _robot* this, int self)
{
	if (this->time_in_action > 1) //busy
		return;
	this->time_in_action = UNLOAD_TIME;
					
	if (this->state == MOTION)
		this->battery.charge -= STOP_MOTION_COST;
	this->state = STOP;
					
    if (warehouse.room[this->y][this->x] == CELL_OUT && this->loaded == true)
    {
        this->loaded = false;
		if (this->cur_ori == VER)
			warehouse.robots_next_step[this->y][this->x] = CELL_EMPT_ROBOT_VER;
		else
			warehouse.robots_next_step[this->y][this->x] = CELL_EMPT_ROBOT_HOR;
                        
		++this->boxes_delivered;
						
		if (this->battery.charge < TIME_TO_CHARGE_THRESHOLD)
			AssignDest(this, CELL_CHARGER);
		else
			AssignDest(this, CELL_IN);
		
		CurMove[self-1] = 'O';
    }	
}