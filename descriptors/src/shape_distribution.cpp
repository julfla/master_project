#include "descriptors/shape_distribution.h"
#include <math.h>  // provides acos function
#include <pcl/io/pcd_io.h>
#include <pcl/common/distances.h>  // provides pcl::euclideanDistance
#include <algorithm>  // provides sort std::vector::sort
#include <map>
#include <string>
#include <vector>
#include "boost/random.hpp"

std::vector<double> Distribution::compute_sample(DefaultCloud * const cloud,
                                                 int number_points,
                                                 bool scale_coordinates) {
    if (cloud->empty())
        return std::vector<double>();
    float x, y, z;
    x = y = z = 0;
    if (scale_coordinates) {
        for (int i = 0; i < cloud->size(); ++i) {
            x = std::max(x, std::abs(cloud->at(i).x));
            y = std::max(y, std::abs(cloud->at(i).y));
            z = std::max(z, std::abs(cloud->at(i).z));
        }
    }
    std::vector<double> sample;
    boost::minstd_rand generator;
    boost::uniform_int<> uni_dist;
    generator = boost::minstd_rand(std::time(0));
    uni_dist = boost::uniform_int<>(0, cloud->size() - 1);

    for (int i = 0; i < number_points; ++i) {
        // get two random points and compute the distance between them
        pcl::PointXYZ pts1 = cloud->at(uni_dist(generator));
        pcl::PointXYZ pts2 = cloud->at(uni_dist(generator));
        if (scale_coordinates) {
            pts1.x /= x;
            pts2.x /= x;
            pts1.y /= y;
            pts2.y /= y;
            pts1.z /= z;
            pts2.z /= z;
        }
        sample.push_back(static_cast<double>(
                             pcl::euclideanDistance(pts1, pts2)));
    }
    return sample;
}

std::vector<double> Distribution::compute_a3_sample(DefaultCloud * const cloud,
                                                    int number_points) {
    std::vector<double> sample;
    if (cloud->empty())
        return sample;
    boost::minstd_rand generator;
    boost::uniform_int<> uni_dist;
    generator = boost::minstd_rand(std::time(0));
    uni_dist = boost::uniform_int<>(0, cloud->size() - 1);
    for (int i = 0; i < number_points; ++i) {
        // get two random points and compute the distance between them
        pcl::PointXYZ pcl_pts1 = cloud->at(uni_dist(generator));
        pcl::PointXYZ pcl_pts2 = cloud->at(uni_dist(generator));
        pcl::PointXYZ pcl_pts3 = cloud->at(uni_dist(generator));
        Point_3D pts1(pcl_pts1.x, pcl_pts1.y, pcl_pts1.z);
        Point_3D pts2(pcl_pts2.x, pcl_pts2.y, pcl_pts2.z);
        Point_3D pts3(pcl_pts3.x, pcl_pts3.y, pcl_pts3.z);

        // we consider the angle pts2, pts1, pts3
        double d_12 = pts1.squared_distance(pts2);
        double d_13 = pts1.squared_distance(pts3);
        double d_23 = pts2.squared_distance(pts3);

        double cosinus = (d_12 + d_13 - d_23) / (2 * sqrt(d_12) * sqrt(d_13));
        sample.push_back(acos(cosinus));
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
