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

    key_t writer_key;

    // creating the shared memory
    writer_key = ftok("testsharedmemory", 'B');

    // creating the shared memory
    int shmid = shmget(writer_key, MAXSIZE, 0666 | IPC_CREAT);
    if(shmid == FAILURE){
        report_and_exit("shmget failed");
    }

    // attaching the shared memory
    void *shared_memory = shmat(shmid, NULL, 0);
    if(shared_memory == (void *) FAILURE){
        report_and_exit("shmat failed");
    }
    
    printf("shared memory attached at %p\n", shared_memory);

    // writing to the shared memory
    printf("Enter some text:\n");
    fflush(stdout); 
    char buf[MAXSIZE];       
    fgets(buf, MAXSIZE, stdin);

    size_t len= strlen(buf);
    if(len > 0 && buf[len-1] == '\n'){
        buf[len-1] = '\0';
    }

    strcpy(shared_memory, buf);

    // detaching the shared memory
    if(shmdt(shared_memory) == -1){
        report_and_exit("shmdt failed");
    }

    return SUCCESS;
}