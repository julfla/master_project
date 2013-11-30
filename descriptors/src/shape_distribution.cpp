#include "shape_distribution.h"

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
