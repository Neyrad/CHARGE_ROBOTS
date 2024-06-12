#include "model.h"

bool RQ_isFull(struct _robot* robot)
{
	return (robot->commands.front == robot->commands.rear + 1) || (robot->commands.front == 0 && robot->commands.rear == SIZE - 1);
}

bool RQ_isEmpty(struct _robot* robot)
{
	return robot->commands.front == -1;
}

void RQ_enQueue(struct _robot* robot, message_type element)
{
	if (RQ_isFull(robot))
	{
		printf("Queue is full !!!\n");
		assert(false);
	}
	else 
	{
		if (robot->commands.front == -1)
			robot->commands.front = 0;
		robot->commands.rear = (robot->commands.rear + 1) % SIZE;
		robot->commands.items[robot->commands.rear] = element;
		robot->commands.n_elems++;
	}
}

message_type RQ_deQueue(struct _robot* robot)
{
	if (RQ_isEmpty(robot))
	{
		printf("Queue is empty !!!\n");
		printf("robot->destination = (%d, %d)\n", robot->destination.x, robot->destination.y);
		printf("robot              = (%d, %d)\n", robot->x, robot->y);
		assert(false);
		return NOP;
	}
	else
	{
		message_type element = robot->commands.items[robot->commands.front];
		if (robot->commands.front == robot->commands.rear)
		{
			robot->commands.front = -1;
			robot->commands.rear = -1;
		} 

		else
			robot->commands.front = (robot->commands.front + 1) % SIZE;
		
		robot->commands.n_elems--;
		return element;
	}
}

void displayRobotCommands(struct _robot* robot)
{
	if (RQ_isEmpty(robot))
		printf("Empty Queue\n");
	else
	{
		for (int i = robot->commands.front; i != robot->commands.rear; i = (i + 1) % SIZE)
			 PrintCommand(robot->commands.items[i]);
		PrintCommand(robot->commands.items[robot->commands.rear]);
		printf("\n");
	}
}

void PrintCommand(message_type cmd)
{
	switch(cmd)
	{
		case MOVE_U:
			printf("U ");
			break;
		case MOVE_D:
			printf("D ");
			break;
		case MOVE_L:
			printf("L ");
			break;
		case MOVE_R:
			printf("R ");
			break;
		case NOP:
			printf("N ");
			break;
		case LOAD:
			printf("I ");
			break;
		case UNLOAD:
			printf("O ");
			break;
		case CHARGE:
			printf("C ");
			break;
		default:
			printf("%d ", cmd);
			break;
	}
}

void RQ_Init(struct _robot* robot)
{	
	robot->commands.front   = -1;
	robot->commands.rear    = -1;
	robot->commands.n_elems = 0;
}

int RQ_Get_N_Elems(struct _robot* robot)
{
	return robot->commands.n_elems;
}