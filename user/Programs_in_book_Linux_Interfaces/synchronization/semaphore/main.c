#include <sys/wait.h>
#include <unistd.h>
#include <stdio.h>
#include <pthread.h>
#include <stdlib.h>
#include <errno.h>
#include <semaphore.h>
#include <string.h>

#define SUCCESS  1
#define FAILURE -1

// #define DEBUG

#define MAXSIZE 10

void report_and_exit(const char *msg) {
    perror(msg);
    exit(FAILURE);
}

int buf[MAXSIZE];
sem_t sem_odd,sem_even;
sem_t sem_odd_even_done;

void *odd_function(){    
    for(int i=1;i<MAXSIZE;i+=2){
        sem_wait(&sem_odd);       
        buf[i]=i;
        #ifdef DEBUG
        printf("In odd function buf[%d] = %d\n",i,buf[i]);
        #endif // DEBUG
        sem_post(&sem_even);
    }
    #ifdef DEBUG
    sem_post(&sem_odd_even_done);
    #endif
    pthread_exit(NULL);
}

void *even_function(){
    for(int i=0;i<MAXSIZE;i+=2){
        sem_wait(&sem_even);       
        buf[i]=i;
        #ifdef DEBUG
        printf(" In even function buf[%d] = %d\n",i,buf[i]);
        #endif // DEBUG
        sem_post(&sem_odd);
    }
    #ifdef DEBUG 
    sem_post(&sem_odd_even_done);   
    #endif
    pthread_exit(NULL);
}

void * consumer_function(){
    printf("In consumer function\n");
    sem_wait(&sem_odd_even_done);
    sem_wait(&sem_odd_even_done);

    for(int i=0;i<MAXSIZE;i++){
        printf("In consumer function buf[%d] = %d\n",i,buf[i]);
    }
   
    pthread_exit(NULL);
}

int main(){
    pthread_t odd,even;
    pthread_t consumer; 

    //initializing the semaphores
    sem_init(&sem_odd,0,0);
    sem_init(&sem_even,0,1);    
    sem_init(&sem_odd_even_done,0,0);

    //creating the threads
    if (pthread_create(&odd, NULL, odd_function, NULL) != 0) {
       report_and_exit("Error creating thread 1");
    }
    if (pthread_create(&even, NULL, even_function, NULL) != 0) {
       report_and_exit("Error creating thread 2");
    }
    if (pthread_create(&consumer, NULL, consumer_function, NULL) != 0) {
       report_and_exit("Error creating thread 3");
    }

    //join the threads
    if (pthread_join(odd, NULL) != 0) {
       report_and_exit("Error joining thread 1");
    }
    #ifndef DEBUG 
    sem_post(&sem_odd_even_done);
    #endif
    if (pthread_join(even, NULL) != 0) {
       report_and_exit("Error joining thread 2");
    }   
    #ifndef DEBUG
    sem_post(&sem_odd_even_done);
    #endif // !DEBUG
    if (pthread_join(consumer, NULL) != 0) {    
       report_and_exit("Error joining thread 3");
    }

    //destroying the semaphores
    sem_destroy(&sem_odd);
    sem_destroy(&sem_even);
    sem_destroy(&sem_odd_even_done);


    return SUCCESS;
}