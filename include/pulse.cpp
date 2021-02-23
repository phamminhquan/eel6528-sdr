#include "pulse.h"

/* Generate a unit-energy truncated RRC impulse response for pulse shaping
    Assume D <= U <= 2*D
    This RRC pulse is sampled at U*symbol_rate.
    Excess bandwidth = U/D - 1
    sampling rate / symbol rate =  U/D
    Length of impulse response = 2*len+1
*/
void rrc_pulse(std::complex<float>* h, int len, int U, int D)
{
   float beta = float(U - D)/D; //roffoff factor
    h[len] = 1.0-beta+4.0*beta/M_PI;
    float scale = std::norm(h[len]);
    for (int n=1; n<=len; n++) {
        if (n == U/beta/4.0) {
            h[len+n] = beta/sqrt(2.0)*((1.0+2.0/M_PI)*sin(M_PI/4.0/beta)+(1.0-2.0/M_PI)*cos(M_PI/4.0/beta));
        } else {
            h[len+n] = (sin(n*M_PI*(1.0-beta)/U) + 4.0*n*beta/U*cos(n*M_PI*(1.0+beta)/U))*U/n/M_PI/(1.0-16.0*n*n*beta*beta/U/U);
        }
        h[len-n] = h[len+n];
        scale += 2.0*std::norm(h[len+n]);
    }
    scale = sqrt(scale);
    for (int n=0; n<2*len+1; n++) {
        h[n] /=  scale;
    }
}