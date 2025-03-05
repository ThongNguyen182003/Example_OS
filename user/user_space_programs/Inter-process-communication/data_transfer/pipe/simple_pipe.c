/*

1 Simple pipe example

Show how pipes can be used for data transfer between the process parent and
child processes.

Usage : ./simple_pipe "hello from userspace"

Output :Read from pipe in parent : hello from userspace
        Write to pipe from childern : hello from userspace
*/
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h> 
#include <sys/wait.h>

#define SUCCESS 0
#define FAILURE -1

#define ReadEnd 0
#define WriteEnd 1

#define BUF_SIZE 30

void report_and_exit(const char *msg)
{
    perror(msg);
    exit(FAILURE);
}
int main(int argc ,char *argv[])
{
    int fd[2];  //Pipe file descriptor
    char buf[BUF_SIZE];

    if(argc != 2 || strcmp(argv[1] ,"--help") == 0){
        printf("Usage : %s string\n",argv[0]);
        exit(1);
    }

    // Create pipe
    if(pipe(fd) == -1){
        report_and_exit("pipe failed");
    }

    switch (fork()){
        case -1:
            report_and_exit("fork failed");
            break;
        case 0:
            // Child process
            close(fd[ReadEnd]);
            write(fd[WriteEnd],argv[1],BUF_SIZE);
            printf("Write to pipe from childern : %s\n",argv[1]);
            close(fd[WriteEnd]);
            exit(SUCCESS);
        default:
            // Parent process
            close(fd[WriteEnd]);
            read(fd[ReadEnd],buf,BUF_SIZE);
            close(fd[ReadEnd]);
            printf("Read from pipe in parent : %s\n",buf);
            wait(NULL); // Wait for childern to finish
            exit(SUCCESS);
    }
    return SUCCESS;
}