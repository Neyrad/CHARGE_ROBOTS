//The header file template for a ROSS model
//This file includes:
// - the state and message structs
// - extern'ed command line arguments
// - custom mapping function prototypes (if needed)
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

#define CELL_ROBOT_UP    8
#define CELL_ROBOT_RIGHT 6
#define CELL_ROBOT_LEFT  4
#define CELL_ROBOT_DOWN  2

#define CELL_ROBOT_WITH_BOX_UP    88
#define CELL_ROBOT_WITH_BOX_RIGHT 66
#define CELL_ROBOT_WITH_BOX_LEFT  44
#define CELL_ROBOT_WITH_BOX_DOWN  22

#define CELL_CONTAINER 3
#define CELL_BOX       5
#define CELL_WALL      1
#define CELL_EMPTY     0

#define MAX_ROOM_HEIGHT 100
#define MAX_ROOM_LENGTH 100

#define MAX_COMMENT_LENGTH 1000

#define NONE -1

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

struct room
{
    int data[MAX_ROOM_HEIGHT][MAX_ROOM_LENGTH];
    int height;
    int length;
};

struct room storage;

struct cell
{
    int x;
    int y;
    int value;
};

#define MAX_ROBOTS 100

struct _robot
{
    int x;
    int y;
	bool carries_box;
	bool stuck;
	bool unstucking;
	dir direction;
    
	enum
	{
		STOP,
		MOTION,
		ROT_LEFT,
		ROT_RIGHT,
		BOX_IN,
		BOX_OUT,
	} state;
	
	struct cell dest;
	dir dest_dir;
};

struct _robots
{
    struct _robot data[MAX_ROBOTS];
    int N;
};

struct _robots Robots;


//Example enumeration of message type... could also use #defines
typedef enum
{
    ROTATE_LEFT,
    ROTATE_RIGHT,
    MOVE,
    BOX_GRAB,
    BOX_DROP,
    RECEIVED,
    EXECUTED,
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
    int got_msgs_ROTATE_LEFT;
    int got_msgs_ROTATE_RIGHT;
    int got_msgs_MOVE;
    int got_msgs_BOX_GRAB;
    int got_msgs_BOX_DROP;
    int got_msgs_RECEIVED;
    int got_msgs_EXECUTED;
    int got_msgs_INIT;
	int got_msgs_NOP;

    lp_type type;
    double value;

} state;


//Command Line Argument declarations
extern unsigned int setting_1;

//Global variables used by both main and driver
// - this defines the LP types
extern tw_lptype model_lps[];

//Function Declarations
// defined in driver.c:
extern void model_init(state* s, tw_lp* lp);
extern void model_event(state* s, tw_bf* bf, message* in_msg, tw_lp* lp);
extern void model_event_reverse(state* s, tw_bf* bf, message* in_msg, tw_lp* lp);
extern void model_final(state* s, tw_lp* lp);
// defined in map.c:
extern tw_peid model_map(tw_lpid gid);
extern tw_lpid model_typemap (tw_lpid gid);

/*
//Custom mapping prototypes
void model_cutom_mapping(void);
tw_lp * model_mapping_to_lp(tw_lpid lpid);
tw_peid model_map(tw_lpid gid);
*/

extern void parse(const char* path);
extern void PrintMap(const char* log_folder_path);
extern void PrintNSteps(const char* log_folder_path);
extern void RobotsInit();
extern void RobotsPrint();
extern void SendMessageContents(tw_lpid receiver, tw_lp* lp, double ts, lp_type type, double contents);
extern void SendMessage(tw_lpid receiver, tw_lp* lp, double ts, lp_type type);
extern void AssignDest(struct _robot* robot, int goal);
extern int EveryoneResponded(int* arr, int N);
extern int CalcNextMove(struct _robot* robot);
extern int Rotate(int cur, int des);
extern int unstuck(struct _robot* me);
extern bool NeighborAlsoStuck(struct _robot* This);

#endif
