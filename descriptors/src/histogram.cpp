#include "histogram.h"
#include <iostream>

std::vector<double> Histogram::compute_data(const std::vector<double> *sorted_data, int n_bins, const it_dbl begin, const it_dbl end, double min_value, double step ) {
    if(n_bins == 1) {
        std::vector<double> result;
        result.push_back(end - begin);
        return result;
    }
    int offset = n_bins % 2;
    n_bins /= 2;
    double mid_value = min_value + n_bins * step;
    it_dbl mid = std::upper_bound(begin, end, mid_value);

    std::vector<double> left_data = compute_data(sorted_data, n_bins, begin, mid, min_value, step);
    std::vector<double> right_data = compute_data(sorted_data, n_bins + offset, mid, end, min_value + n_bins * step, step);
    left_data.insert(left_data.end(), right_data.begin(), right_data.end());
    return left_data;
}
