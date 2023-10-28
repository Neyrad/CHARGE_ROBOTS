//The header file template for a ROSS model
//This file includes:
// - the state and message structs
// - all function prototypes
// - any other needed structs, enums, unions, or #defines

#ifndef _model_h
#define _model_h

#include "ross.h"
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <ctype.h>
#include <assert.h>
#include <time.h>

#define N_BOXES 3
#define N_CONTAINERS 9

#define CELL_ROBOT_VER    		2
#define CELL_ROBOT_HOR 			4
#define CELL_ROBOT_WITH_BOX_VER 6
#define CELL_ROBOT_WITH_BOX_HOR 8

#define CELL_EMPTY     0
#define CELL_WALL      1
#define CELL_CONTAINER 3
#define CELL_BOX       5
#define CELL_CHARGER   7

#define MAX_ROOM_HEIGHT 100
#define MAX_ROOM_LENGTH 100
#define MAX_COMMENT_LENGTH 1000
#define MAX_ROBOTS 100
//#define MAX_INPUT_LENGTH 20000000
#define MAX_INPUT_LENGTH 2000000

/*------------------ENERGY COST DEFINES------------------*/
#define BATTERY_CAPACITY		152000
#define START_MOTION_COST		    44
#define KEEP_MOTION_COST		     7
#define STOP_MOTION_COST		     0
#define ROTATE_COST				    16
#define TIME_TO_CHARGE_THRESHOLD 51200
#define CHARGE_CHUNK			    64 // charge per time unit


/*-------------------TIME COST DEFINES-------------------*/
#define ROTATE_TIME 3 	// 1.2 sec
#define MOVE_TIME 1		
#define BOX_GRAB_TIME 2
#define BOX_DROP_TIME 1

#define MAX_CYCLES_LiFePO4      2000
#define MAX_CYCLES_LiNiMnCoO2   1500
#define MAX_CYCLES_LeadAcid     220
#define MAX_CYCLES_LiCoO2       680
#define MAX_CYCLES_OF_ALL_TYPES 2000 // put here the largest number of the above

// each step is 0.4 sec
// since ROBOT_VELOCITY = 2.5 m/s &&
//		1 (m/step) / ROBOT_VELOCITY (m/s) = 0.4 (sec/step)
// 9000 * 0.4 = 3600
// => 1 hr of real time is 9000 simulation steps
#define GLOBAL_TIME_END (9000 * 24 * 30)

typedef enum
{
    FALSE = 0,
    TRUE  = 1,
} bool;

typedef enum
{
	UP    = 8,
	DOWN  = 2,
	LEFT  = 4,
	RIGHT = 6,
} dir;

struct point
{
	float x;
	float y;
};

struct _storage
{
    int room  [MAX_ROOM_HEIGHT][MAX_ROOM_LENGTH];
	int robots[MAX_ROOM_HEIGHT][MAX_ROOM_LENGTH];
	int height;
	int length;
};

struct _storage storage;

//BOX - CONTAINER pairs
struct _pairs
{
	//int data[MAX_INPUT_LENGTH][2];
	int** data;
	int cur;
	int length;
	bool eof;
};

struct _pairs pairs;

struct cell
{
    int x;
    int y;
    int value;
};

typedef enum
{
	HOR = 0,
	VER = 1,
} ori;

typedef enum
{
	LiFePO4    = 0,
	LiNiMnCoO2 = 1,
	LeadAcid   = 2,
	LiCoO2     = 3,
} BatteryType;

struct _battery
{
	bool charging;
	//bool dead;
	
	int times_recharged;
	int time_spent_charging;
	
	int charge;
	int capacity;
	
	int BDM[MAX_CYCLES_OF_ALL_TYPES];
	int BDM_cur;
	
	BatteryType type;
};

struct _robot
{
    int x;
    int y;

	int time_in_action;
	bool carries_box;
	struct cell dest;
	
	ori orientation;
	ori dest_ori;
	
	struct _battery battery;

	int pair[2];
	
	enum
	{
		STOP,
		MOTION,
	} state;	
};

struct _robots
{
    struct _robot data[MAX_ROBOTS];
    int N;
};

struct _robots Robots;

typedef enum
{
    ROTATE,
    MOVE_U,
	MOVE_D,
	MOVE_L,
	MOVE_R,
    BOX_GRAB,
    BOX_DROP,
    RECEIVED,
	INIT,
	NOP,
} message_type;

typedef enum
{
    COMMAND_CENTER,
    ROBOT,
} lp_type;

//Message struct
//   this contains all data sent in an event
typedef struct
{
    message_type type;
    double contents;
    tw_lpid sender;
} message;

//State struct
//   this defines the state of each LP
typedef struct
{
    int got_msgs_ROTATE;
    int got_msgs_MOVE_U;
	int got_msgs_MOVE_D;
	int got_msgs_MOVE_L;
	int got_msgs_MOVE_R;
    int got_msgs_BOX_GRAB;
    int got_msgs_BOX_DROP;
    int got_msgs_RECEIVED;
	int got_msgs_INIT;
	int got_msgs_NOP;
	
	int boxes_delivered;

    lp_type type;
    double value;
} state;

extern const tw_optdef model_opts[];

//Command Line Argument declarations
extern unsigned int setting_1;

//Function Declarations
// defined in driver.c:
extern void model_init(state* s, tw_lp* lp);
extern void model_event(state* s, tw_bf* bf, message* in_msg, tw_lp* lp);
extern void model_event_reverse(state* s, tw_bf* bf, message* in_msg, tw_lp* lp);
extern void model_final(state* s, tw_lp* lp);
// defined in map.c:
extern tw_peid model_map(tw_lpid gid);
extern tw_lpid model_typemap (tw_lpid gid);

//Global variables used by both main and driver
// - this defines the LP types
extern tw_lptype model_lps[];

// Converts .csv into 2D int array
extern void Parse(const char* path, int arr[MAX_ROOM_HEIGHT][MAX_ROOM_LENGTH]);
extern void PairsInit(const char* path);

// Frees the memory
extern void FreePairs();

// Prints a snapshot .csv
extern void PrintMap(const char* log_folder_path);

// Prints Number_of_Steps.txt file 
extern void PrintNSteps(const char* log_folder_path);

extern void RobotsInit();

// Prints to the console coords of each robot
extern void RobotsPrint();

extern void SendMessageContents(tw_lpid receiver, tw_lp* lp, double ts, lp_type type, double contents);
extern void SendMessage(tw_lpid receiver, tw_lp* lp, double ts, lp_type type);

// Assign BOX, CONTAINER or CHARGER to the robot as destination
extern void AssignDest(struct _robot* robot, int goal);

// Get a BOX - CONTAINER pair from the 2D int array
extern void GetPair(struct _robot* robot);

// Each robot received a command
extern bool EveryoneResponded(int* arr, int N);

// Simple robot routing (occasionally they run into the central wall)
extern int CalcNextMove(struct _robot* robot);

// Circular robot routing (works properly)
extern int CalcNextMove2(struct _robot* robot);

// Unit test for the ParsePairs() func
extern void PrintPairs();

// Battery Degradation Model curve init
extern void InitBDM(struct _robot* robot, BatteryType BT);

extern void PrintBDM(struct _robot* robot, const char* print_to);
extern int CalculateCapacity(struct _robot* robot);

extern void SimulateROSS(int argc, char* argv[]);
extern void ValidateMacros();
extern void FilesInit();
extern void InitROSS();
extern void Free();
extern void FinalizeROSS();

#endif
