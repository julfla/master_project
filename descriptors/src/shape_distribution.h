#ifndef SHAPE_DISTRIBUTION_H
#define SHAPE_DISTRIBUTION_H

#include <vector>
#include <iostream> // to write into file
#include <cmath>

// include headers that implement a archive in simple text format
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/vector.hpp>

#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "mesh.hpp"
#include "histogram.h"

#define _SAMPLE_LENGTH_ 100000


class Distribution
{
    friend class boost::serialization::access; //used to get access to the private attributes
public:

    //used when loading archive
    Distribution() {}

    Distribution(Mesh mesh) {
        std::vector<double> sample = compute_sample(&mesh, _SAMPLE_LENGTH_);
        assert(sample.size() == _SAMPLE_LENGTH_);
        compute_histogram(sample);
    }

    Distribution(pcl::PointCloud<pcl::PointXYZ> * const cloud) {
        std::vector<double> sample = compute_sample(cloud, _SAMPLE_LENGTH_);
        compute_histogram(sample);
    }

    std::string to_csv();

    std::vector<double> const getDistribution() {
        return distribution;
    }

    void setDistribution(std::vector<double> distribution) {
        this->distribution = distribution;
    }

    typedef double (* double_distance)(double, double);

    // return the sums of the differance between repartion functions
    template <double_distance d >
    double distance(Distribution other_distribution) {

        assert(other_distribution.distribution.size() == distribution.size());
        double sum = 0;
        for(int i = 0 ; i < distribution.size(); ++i)
        {
            double p = this->distribution.at(i);
            double q = other_distribution.getDistribution().at(i);
            sum += d(p,q);
        }
        return sum;
    }

    static Distribution load_archive(const char* path);
    static Distribution load_archive(const std::string path);

    void save_archive(const std::string path);
    void save_archive(const char* path);

private:

    //build the histogram and recopy the data with different resolution
    void compute_histogram(std::vector<double> sample);

    // generate <number_poits> the random var distance between 2 random points
    std::vector<double> compute_sample(Mesh * const mesh, int number_points);
    std::vector<double> compute_sample(pcl::PointCloud<pcl::PointXYZ> * const cloud, int number_points);


    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & this->distribution;
    }

    std::vector<double> distribution;
};

#endif // SHAPE_DISTRIBUTION_H
