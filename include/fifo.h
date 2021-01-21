#ifndef QUEUE_H
#define QUEUE_H


#include <mutex>
#include <list>

// template for type of data in FIFO
template<class T>
class tsFIFO {
    private:
        // FIFO las list type
        std::list<T> queue;
        // mutex to ensure thread sage
        std::recursive_mutex mtx;
    public:
        // helper functions
        void push(T data) {
            // lock mutex so no other thread can use this FIFO
            std::lock_guard<std::recursive_mutex> lock(mtx);
            // check if FIFO is 
            queue.push_back(data);
        }
        
        bool pop(T& data) {
            // lock mutex so no other thread can use this FIFO
            std::lock_guard<std::recursive_mutex> lock(mtx);
            // check if queue is empty
            bool empty_f = queue.empty();
            if (!queue.empty()) {
                data = queue.front();
                queue.pop_front();
            }
            return !empty_f;
        }

        int size(void) {
            // lock mutex so no other thread can use this FIFO
            std::lock_guard<std::recursive_mutex> lock(mtx);
            return queue.size();
        }
};


#endif //QUEUE_H
