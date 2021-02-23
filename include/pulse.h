#ifndef PULSE_H
#define PULSE_H

#include <complex>

// function to generate root-raised cosine pulse
void rrc_pulse(std::complex<float>* h, int len, int U, int D);


#endif