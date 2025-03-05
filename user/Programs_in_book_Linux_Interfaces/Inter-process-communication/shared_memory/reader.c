#include <sys/wait.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>


#define SUCCESS  1
#define FAILURE -1

#define MAXSIZE 1024

void report_and_exit(const char *msg) {
    perror(msg);
    exit(FAILURE);
}

int main(){
    
    key_t reader_key;

    // creating the shared memory
    reader_key = ftok("testsharedmemory", 'B');

    // creating the shared memory
    int shmid = shmget(reader_key, MAXSIZE, 0666 | IPC_CREAT);
    if(shmid == FAILURE){
        report_and_exit("shmget failed");
    }

    // attaching the shared memory
    void *shared_memory = shmat(shmid, NULL, 0);
    if(shared_memory == (void *) FAILURE){
        report_and_exit("shmat failed");
    }

    printf("shared memory attached at %p\n", shared_memory);
    // reading from the shared memory
    char buf[MAXSIZE];
    strcpy(buf, (char *)shared_memory);
    printf("Read from shared memory : %s\n", buf);

    // detaching the shared memory
    if(shmdt(shared_memory) == -1){
        report_and_exit("shmdt failed");
    }

    // closing the shared memory
    if(shmctl(shmid, IPC_RMID, NULL) == -1){
        report_and_exit("shmctl failed");
    }

   return SUCCESS;
}