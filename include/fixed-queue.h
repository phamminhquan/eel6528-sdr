#ifndef FIXED_QUEUE_H
#define FIXED_QUEUE_H


#include <mutex>
#include <list>

// template for type of data in FIFO
template<class T>
class FixedQueue {
    private:
        // FIFO las list type
        std::list<T> q;
        // mutex to ensure thread sage
        std::mutex mtx;
        // fixed size of queue
        size_t max_size;
    public:
        // constructor
        FixedQueue (size_t max_len) {
            max_size = max_len;
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
};


#endif //QUEUE_H