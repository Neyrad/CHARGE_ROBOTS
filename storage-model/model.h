//The header file template for a ROSS model
//This file includes:
// - the state and message structs
// - all function prototypes
// - any other needed structs, enums, unions, or #defines

#ifndef _model_h
#define _model_h

#define __DEBUG_PRINT 0

#include "ross.h"
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <ctype.h>
#include <assert.h>
#include <math.h>
#include <time.h>

#define N_INS 10
#define N_OUTS 18
#define N_CHARGERS 18

#define BIG_NUMBER 666

#define SIZE 8 //Reservation Table


#define MAX_ROOM_SIZE_Y 50
#define MAX_ROOM_SIZE_X 50
#define MAX_COMMENT_LENGTH 1000
#define MAX_ROBOTS 100
#define MAX_INPUT_LENGTH 7200000
#define MAX_PATH_TO_LOG_FOLDER 256

/*------------------ENERGY COST DEFINES------------------*/
#define BATTERY_CAPACITY		152000
#define START_MOTION_COST		    0//44
#define KEEP_MOTION_COST		     0//7
#define STOP_MOTION_COST		     0//0
#define ROTATE_COST				    0//16
#define TIME_TO_CHARGE_THRESHOLD 51200
#define CHARGE_CHUNK			    0//64 // charge per time unit


/*-------------------TIME COST DEFINES-------------------*/
#define ROTATE_TIME 1//3 	// 1.2 sec
#define   MOVE_TIME 1//1		
#define   LOAD_TIME 1//2
#define UNLOAD_TIME 1//1

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
#define GLOBAL_TIME_END (9000)

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

typedef struct
{
	int x;
	int y;
} square;

struct _ins
{
	int size;
	square elem[N_INS];
};

struct _outs
{
	int size;
	square elem[N_OUTS];
};

struct _chargers
{
	int size;
	square elem[N_CHARGERS];
};

struct _ins 	 ins;
struct _outs 	 outs;
struct _chargers chargers;

char CurMove[MAX_ROBOTS];

struct _map
{
	int  elem	[MAX_ROOM_SIZE_Y][MAX_ROOM_SIZE_X];
	bool covered[MAX_ROOM_SIZE_Y][MAX_ROOM_SIZE_X];
};

struct _map in_maps[N_INS];
struct _map out_maps[N_OUTS];
struct _map charger_maps[N_CHARGERS];

typedef enum
{
    ROTATE,
    MOVE_U,
	MOVE_D,
	MOVE_L,
	MOVE_R,
    LOAD,
    UNLOAD,
	CHARGE,
    RECEIVED,
	INIT,
	NOP,
} message_type;

struct point
{
	float x;
	float y;
};

struct _warehouse
{
    int room  			[MAX_ROOM_SIZE_Y][MAX_ROOM_SIZE_X];
	int robots			[MAX_ROOM_SIZE_Y][MAX_ROOM_SIZE_X];
	int robots_next_step[MAX_ROOM_SIZE_Y][MAX_ROOM_SIZE_X];
	int size_y;
	int size_x;
};

struct _warehouse warehouse;

//IN - OUT pairs
struct _pairs
{
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

struct _DegradationModel
{
	int elem[MAX_CYCLES_OF_ALL_TYPES];
	int cur;
};

struct _battery
{
	bool charging;
	
	int times_recharged;
	int time_spent_charging;
	
	int charge;
	int capacity;
	
	struct _DegradationModel DegradationModel;

	BatteryType type;
};

typedef struct
{
	message_type items[SIZE];
	int front;
	int rear;
	int n_elems;
} CircularQueueInt;

struct _robot
{
    int x;
    int y;
	
	int num_in_array;

	int boxes_delivered;
	int time_in_action;
	CELL goal_cell;
	
	bool loaded;
	
	ori  cur_ori;
	ori dest_ori;
	
	struct _battery battery;

	int in_num;
	int out_num;
	int charger_num;
	
	enum
	{
		STOP,
		MOTION
	} state;
	
	int time_layer;
	
	CircularQueueInt commands;
	square commands_end;
	square destination;
};

struct _robots
{
    struct _robot elem[MAX_ROBOTS];
    int N;
};

struct _robots Robots;

typedef struct
{
	int elem[MAX_ROOM_SIZE_Y][MAX_ROOM_SIZE_X];
} layer;

typedef struct
{
	layer items[SIZE];
	layer map;
	int front;
	int rear;
} CircularQueueLayers;

CircularQueueLayers ReservationTable;

typedef struct
{
	int X;
	int Y;
	int T;
	void* NextInSolvedPath; //AStar_Node*
	void* Neighbors[5];     //AStar_Node*
} AStar_Node;

typedef struct
{
	AStar_Node* node;
	void* next; //AStarNode_List*
} AStarNode_List;

typedef struct
{
	AStar_Node* CameFrom;
	float       GScore;
	float       FScore;
} NodeDataMap;

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
	int got_msgs_CHARGE;
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
extern tw_lpid model_typemap(tw_lpid gid);

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
extern void InitDegradationModel(struct _robot* robot, BatteryType BT);

extern void PrintDegradationModel(struct _robot* robot, const char* print_to);
extern int CalculateCapacity(struct _robot* robot);

extern void SimulateROSS(int argc, char* argv[]);
extern void ValidateMacros();
extern void FilesInit();
extern void InitROSS();
extern void Free();
extern void FinalizeROSS();
extern void PrintNBoxesDelivered();

extern void FindInsOutsChargers();
extern void InitMaps();
extern void Fill(struct _map* map, square cur, bool firstLaunch);
extern void PrintMapConsole(int  map[MAX_ROOM_SIZE_Y][MAX_ROOM_SIZE_X], int Num);
extern void PrintCovered   (bool cov[MAX_ROOM_SIZE_Y][MAX_ROOM_SIZE_X], int Num);
extern void PrintRoom();
extern bool Valid(square A);
extern bool ValidAlt(square A);
extern void PrintMoves();
extern void PrintMovesInit();
extern void EmergencyFill(struct _robot* robot, square cur);
extern bool ValidEmpty(square A);
extern bool ValidEmptyExcludingCurRobot(struct _robot* robot, square A);
extern void DumpRobots();
extern bool BlockedFromAllSides(int x, int y);
extern square RandomValidSquare();
extern bool AssignEmptyChargerIfPossible(struct _robot* robot);

extern void Rotate(struct _robot* this, int self);
extern void Move(struct _robot* this, char direction, int self); // 'U', 'D', 'L', 'R'
extern void Load(struct _robot* this, int self);
extern void Unload(struct _robot* this, int self);
extern CELL GetNewCellRobot(struct _robot* this);

extern AStarNode_List* AllNodesGSet;
extern float DistanceBetween(int X1, int Y1, int X2, int Y2);
extern void AddToNodeList(AStarNode_List** List, AStar_Node* NodeToAdd, int* LengthPtr);
extern AStar_Node* CreateNode(int X, int Y, int T, AStarNode_List** AllNodesSet);
extern AStarNode_List* FindInNodeList(AStarNode_List* List, AStar_Node* NodeToFind);
extern void RemoveFromNodeList(AStarNode_List** List, AStar_Node* NodeToRemove, int* LengthPtr);
extern void RemoveAllFromNodeList(AStarNode_List** List, bool FreeNodes);
extern AStar_Node* AStar_Find(struct _robot* robot, int StartX, int StartY, int StartT, int EndX, int EndY, NodeDataMap dataMap[MAX_ROOM_SIZE_Y][MAX_ROOM_SIZE_X][SIZE], int waitUntil);
extern void AStar_GetRoute(struct _robot* robot, square Start, square End, int AgentNumber, int waitUntil);
extern float CostOfGoal(int X1, int Y1, int X2, int Y2);
extern bool RT_isFull();
extern bool RT_isEmpty();
extern void InitReservationTable(int map[MAX_ROOM_SIZE_Y][MAX_ROOM_SIZE_X]);
extern void PrintLayer(layer L, int i);
extern void PrintLayerToFile(char* folderPath, layer L, int i);
extern layer CreateLayer(int i);
extern void RT_enQueue(layer element);
extern layer RT_deQueue();
extern void displayReservationTable();
extern void displayReservationTableAlt();
extern void Reserve(int X, int Y, int T, int AgentNumber);
extern int CustomGetMap(int x, int y, int t);
extern int GetTrueMap(int x, int y);
extern square NewSquare(int X, int Y);
extern bool RQ_isFull(struct _robot* robot);
extern bool RQ_isEmpty(struct _robot* robot);
extern void RQ_enQueue(struct _robot* robot, message_type element);
extern message_type RQ_deQueue(struct _robot* robot);
extern void displayRobotCommands(struct _robot* robot);
extern square NewSquare(int x, int y);
extern void PrintCommand(message_type cmd);
extern void PrintRoomAndRobots();
extern void RQ_Init(struct _robot* robot);
extern float GetGScore(AStar_Node* Current, AStar_Node* Neighbor);
extern bool isReserved(int X, int Y, int T);
extern int RQ_Get_N_Elems(struct _robot* robot);
extern bool ValidNeighbor(AStar_Node* TempNodeToFind, int EndX, int EndY);
extern bool isReserved(int X, int Y, int T);
extern int EmptySpaceLayer(int X, int Y, int T);
#endif
