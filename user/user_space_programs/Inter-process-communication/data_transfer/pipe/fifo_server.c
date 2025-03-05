/*
    This file contains server side code for fifo pipe
    it has syncronization probelm  for data transfer between parent and child processes
    which will be solved by using signalling mechanism later on in this tutorial

*/
#include "fifo_pipe.h"

int main(){
    int fd;
    int buf[CHUNKSIZE * BUF_SIZE];

    // creating the pipe
    if (mkfifo(server_fifo,S_IRUSR|S_IWUSR) == -1){
        if(errno != EEXIST){
            report_and_exit("pipe creation failed");
        }       
    }

    // opening the pipe
    if((fd = open(server_fifo,O_WRONLY)) == -1){
        report_and_exit("pipe open failed");
    }

    // writing to the pipe random numbers
    int i;
    for(i=0;i<BUF_SIZE;i++){
        int j;  
        printf("Write to pipe :");     
        for(j=0;j<CHUNKSIZE;j++){
            buf[j]=rand()%100;
            printf("buf[%d] = %d\t",j,buf[j]);
        }
        printf("\n");
        write(fd,buf,CHUNKSIZE);       
        usleep(((rand())%MAXSLEEP_SECONDS)+ 1);     
    }
    // closing the pipe
    close(fd);  
    // unlinking the pipe
    unlink(server_fifo);   
    return SUCCESS;
}