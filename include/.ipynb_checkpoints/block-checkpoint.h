#ifndef BLOCK_H
#define BLOCK_H

#include <vector>

// create a block type which is a pair of block number and vector of samples
template <typename samp_type>
using Block = typename std::pair<int, std::vector<samp_type>>;

#endif