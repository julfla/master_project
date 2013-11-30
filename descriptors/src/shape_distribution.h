#ifndef SHAPE_DISTRIBUTION_H
#define SHAPE_DISTRIBUTION_H

#include <vector>
#include <map>
#include <iostream> // to write into file
#include <algorithm> //provide sort std::vector::sort
#include "point_3d.h"
#include "mesh.h"
#include "vector_3d.h"
#include "histogram.h"
#include <cmath>

// include headers that implement a archive in simple text format
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/vector.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include "boost/random.hpp"

#define _SAMPLE_LENGTH_ 100000


class Distribution
{
    friend class boost::serialization::access; //used to get access to the private attributes
public:

    Distribution() {
        //nothing to do
    }

    Distribution(Mesh mesh, int number_points=0, int sample_length=0) {
        if(number_points != 0)
            std::cerr << "Warning : number of point for distribution is deprecated." << std::endl;
        if (sample_length != 0)
            std::cerr << "Warning : sample length option is deprecated." << std::endl;
        std::vector<double> sample = compute_sample(&mesh, _SAMPLE_LENGTH_);
        assert(sample.size() == _SAMPLE_LENGTH_);
        compute_histogram(sample);
    }

    Distribution(pcl::PointCloud<pcl::PointXYZ> * const cloud) {
        std::vector<double> sample = compute_sample(cloud, _SAMPLE_LENGTH_);
        compute_histogram(sample);
    }

    std::string to_csv() {
        std::stringstream buff;
        for (std::vector<double>::iterator it = distribution.begin(); it < distribution.end(); ++it)
            buff << *it << std::endl;
        return buff.str();
    }

    std::vector<double> getDistribution() {
        return distribution;
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

private:

    void compute_histogram(std::vector<double> sample);
    std::vector<double> compute_sample(Mesh * const mesh, int number_points);
    std::vector<double> compute_sample(pcl::PointCloud<pcl::PointXYZ> * const cloud, int number_points);

    std::vector<double> distribution;

    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & distribution;
    }

};

#endif // SHAPE_DISTRIBUTION_H
