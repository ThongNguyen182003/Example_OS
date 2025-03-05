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

void report_and_exit(const char *msg) {
    perror(msg);
    exit(FAILURE);
}

typedef struct {
    long mtype;
    char mtext[MAX_MSG_SIZE];
} message;


int main(){

    int msg_id;
    int running = 1;

    key_t sender_key = ftok("testqueue", 'B');

    // creating the message queue
    if ((msg_id = msgget(sender_key, 0666 | IPC_CREAT)) == -1){
        report_and_exit("msgget failed");
    }

    // sending messages
    message msg;

    while(running){
        printf("Enter some text:\n");
        fflush(stdout); 
        char buf[MAX_MSG_SIZE];       
        fgets(buf, MAX_MSG_SIZE, stdin);

        size_t len= strlen(buf);
        if(len > 0 && buf[len-1] == '\n'){
            buf[len-1] = '\0';
        }

        strcpy(msg.mtext, buf);
        if (strcmp(msg.mtext, "exit") == 0){
            running = 0;
        }

        // sending message
        // it will tell the type of message and reciver end can recive the message accordingly
        msg.mtype = 2;
        if (msgsnd(msg_id, &msg, MAX_MSG_SIZE, 0) == -1){
            report_and_exit("msgsnd failed");
        }       
    }

    // deleting the message queue
    if (msgctl(msg_id, IPC_RMID, NULL) == -1){
        report_and_exit("msgctl failed");
    }

    printf("Message session has been ended\n");


    return SUCCESS;
}