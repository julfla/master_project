#ifndef SHAPE_DISTRIBUTION_H
#define SHAPE_DISTRIBUTION_H


// include headers that implement a archive in simple text format
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/vector.hpp>

#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "mesh/point_3d.hpp"

#include <vector>
#include <cmath>
#include <string>

#include "descriptors/histogram.h"

#define _SAMPLE_LENGTH_ 100000

class Distribution {
    // Used to get access to the private attributes
    friend class boost::serialization::access;

    typedef pcl::PointXYZ DefaultPoint;
    typedef pcl::PointCloud<DefaultPoint> DefaultCloud;


    public:
    static const int SHAPE_DISTRIBUTION_SIZE = 64;

    // Used when loading archive
    Distribution() {}

    explicit Distribution(DefaultCloud * const cloud) {
        std::vector<double> sample = compute_sample(cloud, _SAMPLE_LENGTH_);
        std::sort(sample.begin(), sample.end());
        Histogram histogram(&sample, SHAPE_DISTRIBUTION_SIZE);
        append_histogram(histogram);
    }

    void append_histogram(const Histogram &histogram) {
        distribution.insert(distribution.end(),
                            histogram.data->begin(),
                            histogram.data->end());
    }

    std::string to_csv();

    std::vector<double> const getDistribution() {
        return distribution;
    }

    void setDistribution(std::vector<double> distribution) {
        this->distribution = distribution;
    }

    // return the sums of the differance between repartion functions
    typedef double (* double_distance)(double, double);
    template <double_distance d >
    double distance(Distribution other_distribution) {
        assert(other_distribution.distribution.size() == distribution.size());
        double sum = 0;
        for (int i = 0 ; i < distribution.size(); ++i) {
            double p = this->distribution.at(i);
            double q = other_distribution.getDistribution().at(i);
            sum += d(p, q);
        }
        return sum;
    }

    static Distribution load_archive(const char* path);
    static Distribution load_archive(const std::string path);

    void save_archive(const std::string path);
    void save_archive(const char* path);

    private:
    // Generate <number_points> the random var distance between 2 random points
    std::vector<double> compute_sample(DefaultCloud * const cloud,
                                       int number_points,
                                       bool scale_coordinates = false);

    // Generate <number_points> the random var 'angle formed by 3 random points'
    std::vector<double> compute_a3_sample(DefaultCloud * const cloud,
                                          int number_points);


    template<class Archive>
    void serialize(Archive & ar, const unsigned int version) {
        ar & this->distribution;
    }

    std::vector<double> distribution;
};

const int Distribution::SHAPE_DISTRIBUTION_SIZE;

#endif  // SHAPE_DISTRIBUTION_H
