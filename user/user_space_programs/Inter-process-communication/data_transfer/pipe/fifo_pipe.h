#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h> 
#include <sys/wait.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>


#define SUCCESS 0
#define FAILURE -1

#define MAXSLEEP_SECONDS 180 

#define BUF_SIZE 30
#define CHUNKSIZE 5


void report_and_exit(const char *msg)
{
    perror(msg);
    exit(FAILURE);
}

const char* server_fifo = "./fifoChannel";