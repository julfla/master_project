#include "histogram.h"

std::vector<int> Histogram::compute_data(std::vector<double> sorted_data, int n_bins, const it_dbl begin, const it_dbl end, double min_value, double max_value ) {
    if(max_value == min_value && max_value == 0)
        max_value == *sorted_data.end();
    if(n_bins == 1) {
        std::vector<int> result;
        result.push_back(end - begin);
        return result;
    }
    else {
        double mid_value = 0.5 * (max_value - min_value);
        it_dbl midle = begin;
        std::binary_search(midle, end, mid_value);
        int offset = n_bins % 2;
        n_bins = (n_bins - offset) / 2;
        std::vector<int> left_data = compute_data(sorted_data, n_bins + offset, begin, midle, *begin, mid_value);
        std::vector<int> right_data = compute_data(sorted_data, n_bins, midle, end, mid_value, *end);
        left_data.insert(left_data.end(), right_data.begin(), right_data.end());
        return left_data;
    }
}
