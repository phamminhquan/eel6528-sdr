#include "power-average.h"

/***********************************************************************
 * Power averager thread function
 **********************************************************************/
void power_average (tsFIFO<Block<std::complex<float>>>& fifo_in, const int thread_number)
{
    // create logger
    Logger logger("Power averager " + std::to_string(thread_number) , "./pow_ave" + std::to_string(thread_number) + ".log");
    // create dummy block
    Block<std::complex<float>> block;
    // create output filestream
    std::ofstream cap_file ("capture.dat", std::ofstream::binary);
    // parameters
    float pow_ave_lin = 0;
    float pow_ave_db = 0;
    int block_size = 0;
    while (not stop_signal_called) {
        if (fifo_in.size() != 0) {
            // reset param
            pow_ave_lin = 0;
            pow_ave_db = 0;
            // pop block from fifo
            fifo_in.pop(block);
            block_size = block.second.size();
            cap_file.write((const char*) block.second.data(), block_size*sizeof(std::complex<float>));
            for (int i=0; i<block_size; i++) {
                pow_ave_lin += std::pow(std::abs(block.second[i]), 2);
            }
            pow_ave_lin /= block_size;
            pow_ave_db = 10*log10(pow_ave_lin);
            // print out average power
            logger.log("Block: " + std::to_string(block.first) +
                    "\tAverage Power: " + std::to_string(pow_ave_db) + " dB");
        }
    }
    // notify user that processing thread is done
    logger.log("Power averager thread is done and closing");
}