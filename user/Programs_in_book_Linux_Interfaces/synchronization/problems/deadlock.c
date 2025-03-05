#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>

// Global mutex variables
pthread_mutex_t mutex1 = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutex2 = PTHREAD_MUTEX_INITIALIZER;

// Thread function 1
void *thread1_function() {
    // Acquire mutex1
    pthread_mutex_lock(&mutex1);
    printf("Thread 1 acquired mutex1\n");

    // Sleep to simulate some work
    sleep(1);

    // Attempt to acquire mutex2
    printf("Thread 1 waiting to acquire mutex2...\n");
    pthread_mutex_lock(&mutex2);
    printf("Thread 1 acquired mutex2\n");

    // Critical section
    // ...

    // Release mutexes
    pthread_mutex_unlock(&mutex2);
    pthread_mutex_unlock(&mutex1);

    pthread_exit(NULL);
}

// Thread function 2
void *thread2_function() {
    // Acquire mutex2
    pthread_mutex_lock(&mutex2);
    printf("Thread 2 acquired mutex2\n");

    // Sleep to simulate some work
    sleep(1);

    // Attempt to acquire mutex1
    printf("Thread 2 waiting to acquire mutex1...\n");
    pthread_mutex_lock(&mutex1);
    printf("Thread 2 acquired mutex1\n");

    // Critical section
    // ...

    // Release mutexes
    pthread_mutex_unlock(&mutex1);
    pthread_mutex_unlock(&mutex2);

    pthread_exit(NULL);
}

int main() {
    pthread_t thread1, thread2;

    // Create thread 1
    if (pthread_create(&thread1, NULL, thread1_function, NULL) != 0) {
        fprintf(stderr, "Error creating thread 1\n");
        return 1;
    }

    // Create thread 2
    if (pthread_create(&thread2, NULL, thread2_function, NULL) != 0) {
        fprintf(stderr, "Error creating thread 2\n");
        return 1;
    }

    // Wait for threads to finish
    if (pthread_join(thread1, NULL) != 0) {
        fprintf(stderr, "Error joining thread 1\n");
        return 1;
    }
    if (pthread_join(thread2, NULL) != 0) {
        fprintf(stderr, "Error joining thread 2\n");
        return 1;
    }

    printf("Both threads have finished\n");

    return 0;
}
