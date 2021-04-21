#ifndef BLOCK_H
#define BLOCK_H

#include <vector>
#include <chrono>

// create a block type which is a pair of block number and vector of samples
//template <typename samp_type>
//using Block = typename std::pair<int, std::vector<samp_type>>;


struct Block_timestamps {
    // rx worker time stamps
    std::chrono::steady_clock::time_point rx_end;
    // iir time stamps
    std::chrono::steady_clock::time_point iir_start;
    std::chrono::steady_clock::time_point iir_end;
    // energy detector time stamps
    std::chrono::steady_clock::time_point ed_start;
    std::chrono::steady_clock::time_point ed_end;
    // match filter time stamps
    std::chrono::steady_clock::time_point mf_start;
    std::chrono::steady_clock::time_point mf_end;
    // agc time stamps
    std::chrono::steady_clock::time_point agc_start;
    std::chrono::steady_clock::time_point agc_end;
    // acq time stamps
    std::chrono::steady_clock::time_point acq_start;
    std::chrono::steady_clock::time_point acq_end;
    // ecc time stamps
    std::chrono::steady_clock::time_point ecc_start;
    std::chrono::steady_clock::time_point ecc_end;
    // arq time stamps
    std::chrono::steady_clock::time_point arq_start;
    std::chrono::steady_clock::time_point arq_end;
};


template <typename samp_type>
struct Block { 
    // ID
    int first;
    // vector of samples
    std::vector<samp_type> second;
    // time stamps
    Block_timestamps ts;
}; 

#endif