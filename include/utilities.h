#ifndef UTILITIES_H
#define UTILITIES_H

#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <string>

//! Change to filename, e.g. from usrp_samples.dat to usrp_samples.00.dat,
//  but only if multiple names are to be generated.
std::string generate_out_filename (const std::string& base_fn, size_t n_names, size_t this_name);

#endif