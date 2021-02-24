#ifndef STOP_SIGNAL_H
#define STOP_SIGNAL_H

// declar global ctrl-c signal
extern bool stop_signal_called;

// interrupt function
void sig_int_handler(int);

#endif