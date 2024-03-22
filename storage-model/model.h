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

#define N_INS 3
#define N_OUTS 9

#define BIG_NUMBER 		666
#define MOVES_STUCK_LIMIT 3


#define MAX_ROOM_SIZE_Y 10
#define MAX_ROOM_SIZE_X 10
#define MAX_COMMENT_LENGTH 1000
#define MAX_ROBOTS 100
//#define MAX_INPUT_LENGTH 20000000
#define MAX_INPUT_LENGTH 2000000
#define MAX_PATH_TO_LOG_FOLDER 256

/*------------------ENERGY COST DEFINES------------------*/
#define BATTERY_CAPACITY		152000
#define START_MOTION_COST		    0//44
#define KEEP_MOTION_COST		    0// 7
#define STOP_MOTION_COST		     0
#define ROTATE_COST				    0//16
#define TIME_TO_CHARGE_THRESHOLD 51200
#define CHARGE_CHUNK			    64 // charge per time unit


/*-------------------TIME COST DEFINES-------------------*/
#define ROTATE_TIME 3 	// 1.2 sec
#define   MOVE_TIME 1		
#define   LOAD_TIME 2
#define UNLOAD_TIME 1
#define STALL_TIME  250

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
#define GLOBAL_TIME_END (9000 * 24)

FILE* LogFile;

typedef enum
{
    false = 0,
    true  = 1
} bool;

typedef enum
{
	CELL_EMPT_ROBOT_VER = 2,
	CELL_EMPT_ROBOT_HOR = 4,
	CELL_FULL_ROBOT_VER = 6,
	CELL_FULL_ROBOT_HOR = 8,

	CELL_EMPTY			= 0,
	CELL_WALL			= 1,
	CELL_OUT			= 3,
	CELL_IN				= 5,
	CELL_CHARGER	 	= 7	
} CELL;

struct square
{
	int x;
	int y;
};

struct _ins
{
	int size;
	struct square elem[N_INS];
};

struct _outs
{
	int size;
	struct square elem[N_OUTS];
};

struct _ins  ins;
struct _outs outs;

char CurMove[MAX_ROBOTS];

struct map
{
	int  elem	[MAX_ROOM_SIZE_Y][MAX_ROOM_SIZE_X];
	bool covered[MAX_ROOM_SIZE_Y][MAX_ROOM_SIZE_X];
};

struct map out_maps[N_OUTS];
struct map  in_maps[N_INS];

struct point
{
	float x;
	float y;
};

struct _warehouse
{
    int room  [MAX_ROOM_SIZE_Y][MAX_ROOM_SIZE_X];
	int robots[MAX_ROOM_SIZE_Y][MAX_ROOM_SIZE_X];
	int size_y;
	int size_x;
};

struct _warehouse warehouse;

//IN - OUT pairs
struct _pairs
{
	//int data[MAX_INPUT_LENGTH][2];
	int** elem;
	int cur;
	int length;
	bool eof;
};

struct _pairs pairs;

typedef enum
{
	HOR = 0,
	VER = 1
} ori;

typedef enum
{
	LiFePO4    = 0,
	LiNiMnCoO2 = 1,
	LeadAcid   = 2,
	LiCoO2     = 3
} BatteryType;

struct _battery
{
	bool charging;
	
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

	int boxes_delivered;
	int time_in_action;
	int stuck;
	CELL goal_cell;
	
	bool loaded;
	bool emergency;
	bool escape_flower;
	
	struct square flower_goal;
	
	ori  cur_ori;
	ori dest_ori;
	
	struct _battery battery;

	int in_num;
	int out_num;
	
	enum
	{
		STOP,
		MOTION
	} state;
	
	struct map emergency_map;
};

struct _robots
{
    struct _robot elem[MAX_ROBOTS];
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
    LOAD,
    UNLOAD,
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
    int got_msgs_LOAD;
    int got_msgs_UNLOAD;
    int got_msgs_RECEIVED;
	int got_msgs_INIT;
	int got_msgs_NOP;
	
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
extern void Parse(const char* path, int arr[MAX_ROOM_SIZE_Y][MAX_ROOM_SIZE_X]);
extern void PairsInit(const char* path);

// Frees the memory
extern void FreePairs();

extern void RobotsInit();

// Prints to the console coords of each robot
extern void RobotsPrint();

extern void SendMessageContents(tw_lpid receiver, tw_lp* lp, double ts, lp_type type, double contents);
extern void SendMessage(tw_lpid receiver, tw_lp* lp, double ts, lp_type type);

// Assign IN, OUT or CHARGER to the robot as destination
extern void AssignDest(struct _robot* robot, CELL goal);

// Get a IN - OUT pair from the 2D int array
extern void GetPair(struct _robot* robot);

// Each robot received a command
extern bool EveryoneResponded(int* arr, int N);


extern int CalcNextMove(struct _robot* robot);

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
extern void PrintNBoxesDelivered();

extern void FindInsOuts();
extern void InitMaps();
extern void Fill(struct map* map, struct square cur);
extern void PrintMapConsole(int  map[MAX_ROOM_SIZE_Y][MAX_ROOM_SIZE_X], int Num);
extern void PrintCovered   (bool cov[MAX_ROOM_SIZE_Y][MAX_ROOM_SIZE_X], int Num);
extern void PrintRoom();
extern bool Valid(struct square A);
extern int UnstuckMoveSequence(struct _robot* robot);
extern void PrintMoves();
extern void PrintMovesInit();
extern void EmergencyFill(struct _robot* robot, struct square cur);
extern void EmergencyMapFill(struct _robot* robot);
extern bool ValidEmpty(struct square A);
extern bool ValidEmptyExcludingCurRobot(struct _robot* robot, struct square A);
extern void EmergencyMapInit(struct map* emergency_map);
extern int RandomLegalMove(struct _robot* robot);
extern bool EverybodyWhoIsStuckFailedToGetOut();
extern void DumpRobots();
extern bool BlockedFromAllSides(int x, int y);
extern bool FlowerFormation(struct _robot* robot);
extern bool isFlower(struct square center);
extern struct square RandomValidSquare();
extern bool NoOneInEmergencyMode();

#endif
