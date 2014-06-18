#include "descriptors/shape_distribution.h"
#include <pcl/io/pcd_io.h>
#include <pcl/common/distances.h>  // provides pcl::euclideanDistance
#include <algorithm>  // provides sort std::vector::sort
#include <map>
#include <string>
#include <vector>
#include "boost/random.hpp"

void Distribution::compute_histogram(std::vector<double> sample) {
    std::sort(sample.begin(), sample.end());
    Histogram hist = Histogram(&sample, 64);
    this->distribution.insert(this->distribution.end(),
                              hist.data->begin(), hist.data->end());
}

std::vector<double> Distribution::compute_sample(DefaultCloud * const cloud,
                                                 int number_points) {
    if (cloud->empty())
        return std::vector<double>();
    std::vector<double> sample;
    boost::minstd_rand generator;
    boost::uniform_int<> uni_dist;
    generator = boost::minstd_rand(std::time(0));
    uni_dist = boost::uniform_int<>(0, cloud->size() - 1);

    for (int i = 0; i < number_points; ++i) {
        // get two random points and compute the distance between them
        pcl::PointXYZ pts1 = cloud->at(uni_dist(generator));
        pcl::PointXYZ pts2 = cloud->at(uni_dist(generator));
        sample.push_back(static_cast<double>(
                             pcl::euclideanDistance(pts1, pts2)));
    }
    return sample;
}

Distribution Distribution::load_archive(const std::string path) {
    return load_archive(path.c_str());
}

Distribution Distribution::load_archive(const char* path) {
    Distribution temp;
    std::ifstream ifs(path);
    boost::archive::text_iarchive oa(ifs);
    oa >> temp;
    return temp;
}

void Distribution::save_archive(const char* path) {
    std::ofstream ofs(path);
    boost::archive::text_oarchive oa(ofs);
    oa << *this;
}

void Distribution::save_archive(const std::string path) {
    save_archive(path.c_str());
}


std::string Distribution::to_csv() {
    std::stringstream buff;
    for (std::vector<double>::iterator it = distribution.begin();
         it < distribution.end(); ++it)
        buff << *it << std::endl;
    return buff.str();
}
