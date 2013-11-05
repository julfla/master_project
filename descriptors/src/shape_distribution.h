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
        std::sort(sample.begin(), sample.end());
        Histogram hist = Histogram(&sample, 256);

        this->distribution.insert(this->distribution.end(), hist.data->begin(), hist.data->end());
        hist.scale_down(2);
        this->distribution.insert(this->distribution.end(), hist.data->begin(), hist.data->end());
        hist.scale_down(2);
        this->distribution.insert(this->distribution.end(), hist.data->begin(), hist.data->end());
        hist.scale_down(2);
        this->distribution.insert(this->distribution.end(), hist.data->begin(), hist.data->end());

        //TODO : Find a more efficient way, doing normalization in Histogram !!
        /*for( it_dbl it = distribution.begin(); it < distribution.end(); ++it )
         *it /= _SAMPLE_LENGTH_;
         */
        assert((int) distribution.size() == (32 + 64 + 128 + 256));
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

    std::vector<double> distribution;

    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & distribution;
    }

    std::vector<double> compute_sample(Mesh * const mesh, int number_points) {
        if(mesh->empty())
            return std::vector<double>();
        std::vector<double> sample;
        for (int i=0; i < number_points; ++i) {
            //get two random points and compute the distance between them
            Point_3D pts1 = mesh->retreive_random_point();
            Point_3D pts2 = mesh->retreive_random_point();
            sample.push_back(pts1.distance(pts2));
        }
        return sample;
    }

};

#endif // SHAPE_DISTRIBUTION_H
