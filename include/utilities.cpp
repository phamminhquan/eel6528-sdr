#include "utilities.h"

std::string generate_out_filename(
    const std::string& base_fn, size_t n_names, size_t this_name)
{
    if (n_names == 1) {
        return base_fn;
    }

    boost::filesystem::path base_fn_fp(base_fn);
    base_fn_fp.replace_extension(boost::filesystem::path(
        str(boost::format("%02d%s") % this_name % base_fn_fp.extension().string())));
    return base_fn_fp.string();
}

float correlation (const std::vector<std::complex<float>> vec1,
                   const std::vector<std::complex<float>> vec2) {
    // create a temp vector
    std::complex<float> s=0;
    // multiply/add the vectors
    // assume both vectors same size
    for (int i=0; i<vec1.size(); i++)
        s += vec1[i]*vec2[i];
    return std::abs(s);
}

std::pair<int, float> where_max (const std::vector<float> vec) {
    int vec_size = vec.size();
    std::pair<int, float> max_entry;
    for (int i=0; i<vec_size; i++) {
        if (vec[i] > max_entry.second) {
            max_entry.second = vec[i];
            max_entry.first = i;
        }
    }
    return max_entry;
}

unsigned char to_uchar (const std::vector<bool> input) {
    // create dummy unsigned char
    unsigned char output = 0;
    // loop through 8 bit
    for (int i=0; i<8; i++) {
        if (input[i]) {
            output |= 1 << i;
        }
    }
    return output;
}

std::vector<unsigned char> to_uchar_vec (const std::vector<bool> input) {
    // only take the highest number of bits that is divisible by 8
    int input_size = input.size();
    int output_size = (int)std::floor(input_size/8);
    // create dummy output
    std::vector<unsigned char> output;
    output.resize(output_size);
    // loop through each char size
    for (int i=0; i<output_size; i++) {
        // grab boolean vector
        std::vector<bool> input_slice (input.begin()+i*8, input.begin()+(i+1)*8);
        output[i] = to_uchar(input_slice);
    }
    return output;
}

std::vector<bool> to_bool (const unsigned char input) {
    // create temp vector
    std::vector<bool> temp_vec;
    temp_vec.resize(8);
    std::bitset<8> temp_bitset (input);
    for (int i=0; i<8; i++)
        temp_vec[i] = temp_bitset[i];
    return temp_vec;
}

std::vector<bool> to_bool_vec (const std::vector<unsigned char> input) {
    // create temp vector
    int input_size = input.size();
    int output_size = input_size*8;
    std::vector<bool> output;
    output.resize(output_size);
    std::vector<bool> temp;
    temp.resize(8);
    for (int i=0; i<input_size; i++) {
        temp = to_bool(input[i]);
        for (int k=0; k<8; k++)
            output[i*8+k] = temp[k];
    }
    return output;
}