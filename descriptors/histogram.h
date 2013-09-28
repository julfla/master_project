#ifndef HISTOGRAM_H
#define HISTOGRAM_H

#include <vector>
#include <algorithm>

typedef std::vector<double>::iterator it_dbl;

class Histogram {

    Histogram(std::vector<double> sorted_data, int n_bins) {
        data = compute_data(sorted_data, n_bins, sorted_data.begin(), sorted_data.end());
    }

private:

    std::vector<int> data;
    std::vector<int> compute_data(const std::vector<double> sorted_data, int n_bins, const it_dbl begin, const it_dbl end, double min_value = 0, double max_value = 0 );


};

#endif // HISTOGRAM_H
