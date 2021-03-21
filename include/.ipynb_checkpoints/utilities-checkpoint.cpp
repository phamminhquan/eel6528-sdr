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