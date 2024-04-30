#include "model.h"

void InitDegradationModel(struct _robot* robot, BatteryType BT)
{
	robot->battery.DegradationModel.cur = 0;
	robot->battery.type = BT;
	switch(robot->battery.type)
	{
		case LiFePO4:
		{
			const float A =  0.00000005;
			const float B = -0.000287;
			const float C =  1;
	
			for (int i = 0; i < MAX_CYCLES_LiFePO4; ++i)
				robot->battery.DegradationModel.elem[i] = (A*i*i + B*i + C) * BATTERY_CAPACITY;
		}
			break;
		case LiNiMnCoO2:
		{
			const float A1 =  -0.00022;
			const float B1 =   1;
			const float A2 =  -0.001;
			const float B2 =   1.663;
			const int FUNC_CHANGE_CYCLE = 850;	
	
			for (int i = 0;   i < FUNC_CHANGE_CYCLE && i < MAX_CYCLES_LiNiMnCoO2; ++i)
				robot->battery.DegradationModel.elem[i] = (A1*i + B1) * BATTERY_CAPACITY;
			for (int i = FUNC_CHANGE_CYCLE;            i < MAX_CYCLES_LiNiMnCoO2; ++i)
				robot->battery.DegradationModel.elem[i] = (A2*i + B2) * BATTERY_CAPACITY;
		}
			break;
		case LeadAcid:
		{
			/*
			
			y = k*x + b
			
			k = (y2 - y1) / (x2 - x1)
			b = y1 - k*x1
			
			*/
	
			struct point points[10] = {{0,   1},    {25,  0.99}, {50,  0.92}, \
									   {75,  0.85}, {100, 0.77}, {125, 0.75}, \
									   {150, 0.72}, {175, 0.65}, {200, 0.62}, \
									   {225, 0.6}};
									   
			float k = 0;
			float b = 0;
			int cur_pnt = 0;
	
			for (int i = 0; i < MAX_CYCLES_LeadAcid; ++i)
			{
				if (i == points[cur_pnt].x && cur_pnt < 9)
				{
					k = (points[cur_pnt + 1].y - points[cur_pnt].y) / \
						(points[cur_pnt + 1].x - points[cur_pnt].x);
						
					b = points[cur_pnt].y - k * points[cur_pnt].x;
					
					//printf("InitDegradationModel(): k = %f, b = %f\n", k, b);
					
					++cur_pnt;
				}
				robot->battery.DegradationModel.elem[i] = (k * i + b) * BATTERY_CAPACITY;
			}	
		}
			break;
		case LiCoO2:
		{
			/*
			
			y = k*x + b
			
			k = (y2 - y1) / (x2 - x1)
			b = y1 - k*x1
			
			*/
	
			struct point points[12] = {{0,   1},    {45,  1.003}, {90,  1},     \
									   {430,  0.9}, {450, 0.89},  {500, 0.87},  \
									   {535, 0.85}, {580, 0.81},  {600, 0.765}, \
									   {625, 0.71}, {650, 0.655}, {680, 0.62}    };
									   
			float k = 0;
			float b = 0;
			int cur_pnt = 0;
	
			for (int i = 0; i < MAX_CYCLES_LiCoO2; ++i)
			{
				if (i == points[cur_pnt].x && cur_pnt < 11)
				{
					k = (points[cur_pnt + 1].y - points[cur_pnt].y) / \
						(points[cur_pnt + 1].x - points[cur_pnt].x);
						
					b = points[cur_pnt].y - k * points[cur_pnt].x;
					
					//printf("InitDegradationModel(): k = %f, b = %f\n", k, b);
					
					++cur_pnt;
				}
				robot->battery.DegradationModel.elem[i] = (k * i + b) * BATTERY_CAPACITY;
			}	
		}
			break;
		default:
			printf("InitDegradationModel(): Unknown battery type [%d]\n", robot->battery.type);
			return;
	}
}

int CalculateCapacity(struct _robot* robot)
{
	int MAX_CYCLES = -1;
	switch(robot->battery.type)
	{
		case LiFePO4:
			MAX_CYCLES = MAX_CYCLES_LiFePO4;
			break;
		case LiNiMnCoO2:
			MAX_CYCLES = MAX_CYCLES_LiNiMnCoO2;
			break;
		case LeadAcid:
			MAX_CYCLES = MAX_CYCLES_LeadAcid;
			break;
		case LiCoO2:
			MAX_CYCLES = MAX_CYCLES_LiCoO2;
			break;
		default:
			printf("CalculateCapacity(): Unknown battery type [%d]\n", robot->battery.type);
			return -1;
	}
	
	if (robot->battery.DegradationModel.cur == MAX_CYCLES - 1)
	{
		//BatteryDeath();
		printf("\n\n\nCalculateCapacity(): battery of the robot #??? died...\n\n\n\n");
	}
	//assert(robot->battery.DegradationModel.cur < MAX_CYCLES);
	//assert(robot->battery.DegradationModel.elem[robot->battery.DegradationModel.cur] > 0);
	return robot->battery.DegradationModel.elem[robot->battery.DegradationModel.cur++];
}
