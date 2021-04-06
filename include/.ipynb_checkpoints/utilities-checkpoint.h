#ifndef UTILITIES_H
#define UTILITIES_H

#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <string>
#include <vector>
#include <complex>

//! Change to filename, e.g. from usrp_samples.dat to usrp_samples.00.dat,
//  but only if multiple names are to be generated.
std::string generate_out_filename (const std::string& base_fn, size_t n_names, size_t this_name);

// correlation function used in signal acquisition
float correlation (const std::vector<std::complex<float>> vec1,
                   const std::vector<std::complex<float>> vec2);

// function to find index of vector where it is max
std::pair<int, float> where_max (const std::vector<float> vec);

// function to convert vector of boolean to vector of unsigned char
unsigned char to_uchar (const std::vector<bool> input);
std::vector<unsigned char> to_uchar_vec (const std::vector<bool> input);


#endif