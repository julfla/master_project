#include "shape_distribution.h"

#include <algorithm> //provide sort std::vector::sort
#include <map>
#include "boost/random.hpp"
#include <pcl/io/pcd_io.h>

#include <pcl/common/distances.h> //provides pcl::euclideanDistance


void Distribution::compute_histogram(std::vector<double> sample) {
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

std::vector<double> Distribution::compute_sample(Mesh * const mesh, int number_points) {
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

std::vector<double> Distribution::compute_sample(pcl::PointCloud<pcl::PointXYZ> * const cloud, int number_points) {
    if(cloud->empty())
        return std::vector<double>();
    std::vector<double> sample;
    boost::minstd_rand generator;
    boost::uniform_int<> uni_dist;
    generator = boost::minstd_rand(std::time(0));
    uni_dist = boost::uniform_int<>(0,cloud->size() - 1);

    for (int i=0; i < number_points; ++i) {
        //get two random points and compute the distance between them
        pcl::PointXYZ pts1 = cloud->at(uni_dist(generator));
        pcl::PointXYZ pts2 = cloud->at(uni_dist(generator));
        sample.push_back((double) pcl::euclideanDistance(pts1,pts2));
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
    for (std::vector<double>::iterator it = distribution.begin(); it < distribution.end(); ++it)
        buff << *it << std::endl;
    return buff.str();
}
