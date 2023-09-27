//													  //
//	Generates a .csv file with BOX - CONTAINER pairs  //
//  which is used by the ROSS model as input		  //
//													  //

#include <time.h>
#include <stdlib.h>
#include <stdio.h>

#define LOG_LENGTH 250
#define N_BOXES      3
#define N_CONTAINERS 9

const char* log_path = "log.csv";

int main()
{	
	FILE* f = fopen(log_path, "w");
	srand(time(NULL));
	for (int y = 0; y < LOG_LENGTH; ++y)
        fprintf(f, "%d,%d\n", (rand() % N_BOXES) + 1, (rand() % N_CONTAINERS) + 1);
	fclose(f);
}
