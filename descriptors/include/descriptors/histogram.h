#ifndef HISTOGRAM_H
#define HISTOGRAM_H

#include <vector>
#include <algorithm>
#include <iostream>

typedef std::vector<double>::iterator it_dbl;

class Histogram {

public:

    Histogram(std::vector<double> *sorted_data, int n_bins, bool normalize = false);

    Histogram(std::vector<double> *sorted_data, int n_bins, double min_value, double max_value);

    ~Histogram() {
        delete data;
    }

    //merge two Histograms from the right
    void operator<<(Histogram histogram) {
       this->data->insert(this->data->end(), histogram.data->begin(), histogram.data->end());
    }

    std::vector<double> * data;

    void scale_down(int factor);

private:

    std::vector<double> compute_data(const std::vector<double> *sorted_data, int n_bins, const it_dbl begin, const it_dbl end, double min_value, double step );


};

#endif // HISTOGRAM_H
