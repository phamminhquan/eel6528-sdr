#ifndef POWER_AVERAGE_H
#define POWER_AVERAGE_H

#include <complex>
#include <fstream>
#include "fifo.h"
#include "block.h"
#include "logger.h"
#include "stop-signal.h"

// reference stop signal
//extern static bool stop_signal_called;

// power averaging function
void power_average (tsFIFO<Block<std::complex<float>>>& fifo_in, const int thread_number);


#endif