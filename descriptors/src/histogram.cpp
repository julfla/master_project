#include "histogram.h"
#include <iostream>

Histogram::Histogram(std::vector<double> *sorted_data, int n_bins, bool normalize) {
    double min_value, max_value, step;
    data = new std::vector<double>();
    if(sorted_data->empty()) {
        data->push_back(0);
        return;
    }
    max_value = sorted_data->at(sorted_data->size() - 1);
    min_value = sorted_data->at(0);
    step = (max_value - min_value) / n_bins;
    *data = compute_data(sorted_data, n_bins, sorted_data->begin(), sorted_data->end(), min_value, step);
}

Histogram::Histogram(std::vector<double> *sorted_data, int n_bins, double min_value, double max_value) {
    double step = (max_value - min_value) / n_bins;
    data = new std::vector<double>();
    *data = compute_data(sorted_data, n_bins, sorted_data->begin(), sorted_data->end(), min_value, step);
}

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

void Histogram::scale_down(int factor) {
    if(data->size() % factor != 0) {
        std::cerr << "Could not scale a " << data->size() << " length histogram by " << factor << "." << std::endl;
        return;
    }
    std::vector<double> * new_data = new std::vector<double>();
    double temp_sum = 0;
    int temp_relative_pos = 0;
    for(it_dbl it = data->begin(); it < data->end(); ++it) {
        temp_sum += *it;
        temp_relative_pos++;
        if( temp_relative_pos == factor) {
            new_data->push_back(temp_sum);
            temp_sum = 0;
            temp_relative_pos = 0;
        }
    }
    delete data;
    data = new_data;
}
