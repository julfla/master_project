#include <string>

#include <boost/python.hpp>
#include <boost/python/wrapper.hpp>

// fix a bug when the class is exposed to python
// see this post : http://eigen.tuxfamily.org/dox-devel/group__TopicUnalignedArrayAssert.html
// #define EIGEN_DONT_ALIGN_STATICALLY true
#define EIGEN_DONT_VECTORIZE true
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT true

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include "partial_pointcloud/pointcloud_serialization.hpp"
#include "partial_pointcloud/partial_view.h"

using namespace boost::python;

std::string serialize_pointCloud(DefaultPointCloud & self) {
    std::ostringstream oss;
    boost::archive::text_oarchive archive(oss);
    archive << self;
    return oss.str();
}

void unserialize_pointcloud(DefaultPointCloud & self, std::string data) {
    std::istringstream iss(data);
    boost::archive::text_iarchive archive(iss);
    archive >> self;
}

void save_point_cloud_as_pcd(DefaultPointCloud& self, const char* path) {
    pcl::io::savePCDFileASCII(path, self);
}

DefaultPointCloud read_point_cloud_from_pcd(const char* path) {
    DefaultPointCloud cloud;
    if (pcl::io::loadPCDFile<DefaultPoint> (path, cloud) == -1) {
        std::string error_msg("File " + std::string(path) + " can't be read.");
        throw error_msg;
    }
    return cloud;
}

DefaultPointCloud compute_view_tweak(PartialViewComputer& self, float theta,
                                     float phi) {
    return self.compute_view(theta, phi);
}

BOOST_PYTHON_MODULE(libpypartialview) {
    class_<DefaultPointCloud>("PointCloud")
            .def("size", &DefaultPointCloud::size )
            .add_property("serialized_data", &serialize_pointCloud,
                          &unserialize_pointcloud)
            .def("save_pcd", &save_point_cloud_as_pcd)
            .def("load_pcd", &read_point_cloud_from_pcd)
            .staticmethod("load_pcd");

    class_<PartialViewComputer>("PartialViewComputer")
            .def("display_mesh", &PartialViewComputer::displayMesh )
            .def("load_mesh", &PartialViewComputer::loadMesh )
            .def("compute_view", &PartialViewComputer::compute_view )
            .def("compute_entropy", &PartialViewComputer::compute_entropy);
}
