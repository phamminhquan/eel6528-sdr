#ifndef TIMER_H
#define TIMER_H

#include <mutex>
#include <chrono>
#include <iostream>

using namespace std::chrono;

// template for type of data in FIFO
class Timer {
    private:
        // mutex to ensure thread sage
        std::mutex mtx;
        // start and end time
        steady_clock::time_point start;
        steady_clock::time_point end;
        // time spand duration
        duration<double> dur;
        double dur_count;
    
    public:
        // constructor
        Timer (void) {
            start = steady_clock::now();
            end = steady_clock::now();
            dur = duration_cast<duration<double>>(end-start);
            dur_count = dur.count();
        }
    
        // helper functions
        // function to reset start time to now
        void reset (void) {
            // lock mutex so no other thread can use this
            std::lock_guard<std::mutex> lock(mtx);
            // reset start time and end time to now
            start = steady_clock::now();
        }
        // function to calculate and return time elapsed to now
        double elapse(void) {
            // lock mutex so no other thread can use this FIFO
            std::lock_guard<std::mutex> lock(mtx);
            // set end time to now
            end = steady_clock::now();
            // calculate time elapsed
            dur = duration_cast<duration<double>>(end-start);
            dur_count = dur.count();
            return dur_count;
        }
};


#endif