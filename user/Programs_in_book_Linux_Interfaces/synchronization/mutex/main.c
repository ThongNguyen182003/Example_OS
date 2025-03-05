#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>

int counter = 0; // Shared global variable

pthread_mutex_t mutex_lock;


void report_and_exit(const char *msg) {
    perror(msg);
    exit(1);
}

void * increment() {    
    printf("Incrementing counter...\n");
    int i;
    pthread_mutex_lock(&mutex_lock);
    for (i = 0; i < 10000; ++i) {
        counter++; // Increment the shared variable
    }
    pthread_mutex_unlock(&mutex_lock);
    pthread_exit(NULL);
}

void * decrement() {
    printf("Decrementing counter...\n");
    pthread_mutex_lock(&mutex_lock);
    int i;
    for (i = 0; i < 10000; ++i) {
        counter--; // Decrement the shared variable
    }
    pthread_mutex_unlock(&mutex_lock);
    pthread_exit(NULL);
}

int main() {
    pthread_t thread1, thread2;

    pthread_mutex_init(&mutex_lock, NULL);

    // Create two threads
    if (pthread_create(&thread1, NULL, increment, NULL) != 0) {
       report_and_exit("Error creating thread 1");
    }
    if (pthread_create(&thread2, NULL, decrement, NULL) != 0) {
        report_and_exit("Error creating thread 2");
    }

    // Wait for both threads to finish
    if (pthread_join(thread1, NULL) != 0) {
      report_and_exit("Error joining thread 1");
    }
    if (pthread_join(thread2, NULL) != 0) {
     report_and_exit("Error joining thread 2");
    }

    // Display the final value of the shared variable
    printf("Final value of counter: %d\n", counter);

    return 0;
}
