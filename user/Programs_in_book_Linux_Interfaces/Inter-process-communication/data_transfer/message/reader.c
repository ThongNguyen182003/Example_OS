#include<stdlib.h>
#include<stdio.h>
#include<string.h>
#include<unistd.h>
#include<sys/types.h>
#include<sys/ipc.h>
#include<sys/msg.h>

// max message size allowed
#define MAX_MSG_SIZE 50 


#define SUCCESS  1
#define FAILURE -1

typedef struct {
    long mtype;
    char mtext[MAX_MSG_SIZE];
} message;

void report_and_exit(const char *msg) {
    perror(msg);
    exit(FAILURE);
}


int main(){
    
    int msg_id;
    int running = 1;

    key_t reciver_key = ftok("testqueue", 'B');

    // creating the message queue
    if ((msg_id = msgget(reciver_key, 0666 | IPC_CREAT)) == -1){
        report_and_exit("msgget failed");
    }

    // receiving messages
    while(running){
        message msg;

        // we are reciving message of type 2
        if (msgrcv(msg_id, &msg, MAX_MSG_SIZE, 2, 0) == -1){
            report_and_exit("msgrcv failed");
        }

        if (strcmp(msg.mtext, "exit") == 0){
            running = 0;
        }
        printf("Message received: %s\n", msg.mtext);
        
    }
    printf("session has been ended\n");
    return SUCCESS;
}