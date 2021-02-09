#ifndef FILTER_TAPS_H
#define FILTER_TAPS_H


#include <fstream>
#include <vector>
#include <string>
#include <csignal>
#include "logger.h"

// template for type of data in FIFO
class FilterTaps {
    private:        
        // vector of taps
        std::vector<std::complex<float>> taps;

        // mutex to ensure thread sage
        std::mutex mtx;
    public:
        // constructor
        FilterTaps(std::string filename) {
            // lock mutex so no other thread can use this FIFO
            std::lock_guard<std::mutex> lock(mtx);
            // get taps line by line
            string line;
            ifstream file;
            file.open(filename);
            if (file.is_open()) {
                while (getline(file, line)) {
                    std::complex<float> coeff (std::stof(line), 0);
                    taps.push_back(coeff);
                    //std::cout << "Taps: " << coeff.real() << " + i * " << coeff.imag() << std::endl;
                }
            }
        }
        
        // function to get filter length
        int len(void) {
            // lock mutex so no other thread can use this FIFO
            std::lock_guard<std::mutex> lock(mtx);
            return taps.size();
        }
};


#endif //QUEUE_H
