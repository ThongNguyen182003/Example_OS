#include "fifo_pipe.h"

int main(){
    int fd;
    
    // opening the pipe
    if((fd = open(server_fifo,O_RDONLY)) == -1){
        report_and_exit("pipe open failed");
    }
    
    // reading from the pipe
    int i;
    int buf[CHUNKSIZE * BUF_SIZE];

    for(i=0;i<BUF_SIZE;i++){
        int byte_read = read(fd,buf, CHUNKSIZE * sizeof(int));
        if(byte_read == -1){
            report_and_exit("pipe read failed");
        }
        // printf("byte_read = %d\n",byte_read);
        printf("Read from pipe :");
        for(int j = 0; j<CHUNKSIZE;j++)
             printf("buf[%d] = %d\t",j,buf[j]);
        printf("\n");
    }
    // closing the pipe
    close(fd);    
    // unlinking the pipe
    unlink(server_fifo);

    return SUCCESS;
}