#ifndef FIXED_QUEUE_H
#define FIXED_QUEUE_H


#include <mutex>
#include <deque>
#include <iostream>

// template for type of data in FIFO
template<class T>
class FixedQueue {
    private:
        // mutex to ensure thread sage
        std::mutex mtx;
        // fixed size of queue
        size_t max_size;
    public:
        // attributes
        // FIFO las list type
        std::deque<T> q;
    
        // constructor
        FixedQueue (size_t max_len) {
            max_size = max_len;
            q.resize(max_len);
            q.clear();
        }
    
        // helper functions
        void push (T data) {
            // lock mutex so no other thread can use this FIFO
            std::lock_guard<std::mutex> lock(mtx);
            // check max_size
            if (q.size() == max_size)
                q.pop_front();
            q.push_back(data);
        }

        int size(void) {
            // lock mutex so no other thread can use this FIFO
            std::lock_guard<std::mutex> lock(mtx);
            return q.size();
        }
    
        void print (void) {
            std::cout << "Queue: ";
            for (int i=0; i<max_size; i++) {
                std::cout << std::to_string(std::abs(q[i])) << ", ";
            }
            std::cout << std::endl;
        }
};


#endif //QUEUE_H