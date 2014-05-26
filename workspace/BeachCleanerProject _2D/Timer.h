/*
 * Timer.h
 *
 *  Created on: 23/05/2014
 *      Author: rgap
 */

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>

#ifndef TIMER_H_
#define TIMER_H_

using namespace std;

typedef struct timeval {
  long tv_sec;
  long tv_usec;
} timeval;

class Timer {
private:
    timeval startTime;
public:

    void start(){
        gettimeofday(&startTime, NULL);
    }

    double stop(){
        timeval endTime;
        long seconds, useconds;
        double duration;

        gettimeofday(&endTime, NULL);

        seconds  = endTime.tv_sec  - startTime.tv_sec;
        useconds = endTime.tv_usec - startTime.tv_usec;

        duration = seconds + useconds/1000000.0;

        return duration;
    }

    static void printTime(double duration){
        printf("%5.6f seconds\n", duration);
    }
};
#endif /* TIMER_H_ */
