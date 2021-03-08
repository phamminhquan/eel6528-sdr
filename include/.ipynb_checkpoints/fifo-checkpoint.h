#ifndef FIFO_H
#define FIFO_H


#include <mutex>
#include <list>

// template for type of data in FIFO
template<class T>
class tsFIFO {
    private:
        // FIFO las list type
        std::list<T> queue;
        // mutex to ensure thread sage
        std::mutex mtx;
    public:
        // helper functions
        void push(T data) {
            // lock mutex so no other thread can use this FIFO
            std::lock_guard<std::mutex> lock(mtx);
            // check if FIFO is 
            queue.push_back(data);
        }
        
        bool pop(T& data) {
            // lock mutex so no other thread can use this FIFO
            std::lock_guard<std::mutex> lock(mtx);
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
            std::lock_guard<std::mutex> lock(mtx);
            return queue.size();
        }
        
        void clear(void) {
            // lock mutex so no other thread can use this FIFO
            std::lock_guard<std::mutex> lock(mtx);
            // clear the fifo
            queue.clear();
        }
};


#endif //QUEUE_H
