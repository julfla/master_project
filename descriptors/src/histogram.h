#ifndef HISTOGRAM_H
#define HISTOGRAM_H

#include <vector>
#include <algorithm>
#include <iostream>

typedef std::vector<double>::iterator it_dbl;

class Histogram {

public:

    Histogram(std::vector<double> *sorted_data, int n_bins, bool normalize = false) {
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

    Histogram(std::vector<double> *sorted_data, int n_bins, double min_value, double max_value) {
        double step = (max_value - min_value) / n_bins;
        data = new std::vector<double>();
        *data = compute_data(sorted_data, n_bins, sorted_data->begin(), sorted_data->end(), min_value, step);
    }

    ~Histogram() {
        delete data;
    }

    void operator<<(Histogram histogram) {
       this->data->insert(this->data->end(), histogram.data->begin(), histogram.data->end());
    }

    std::vector<double> * data;

    void scale_down(int factor) {
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

private:

    std::vector<double> compute_data(const std::vector<double> *sorted_data, int n_bins, const it_dbl begin, const it_dbl end, double min_value, double step );


};

#endif // HISTOGRAM_H
