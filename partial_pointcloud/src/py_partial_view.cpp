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

std::string serialize_pointCloud( DefaultPointCloud & self ) {
    std::ostringstream oss;
    boost::archive::text_oarchive archive( oss );
    archive << self;
    return oss.str();
}

void unserialize_pointcloud( DefaultPointCloud & self, std::string data) {
    std::istringstream iss( data );
    boost::archive::text_iarchive archive( iss );
    archive >> self;
}

void save_point_cloud_as_pcd( DefaultPointCloud& self, const char* path) { 
    pcl::io::savePCDFileASCII(path, self); 
}

DefaultPointCloud read_point_cloud_from_pcd( const char* path) {
    DefaultPointCloud cloud;
    if( pcl::io::loadPCDFile<DefaultPoint> (path, cloud) == -1) {
        std::string error_msg("File " + std::string(path) + " can't be read.");
        throw error_msg;
    }
    return cloud;
}

// sometimes init does not go well, this tweak restart the computer to gain in stability
DefaultPointCloud compute_view_tweak_int(PartialViewComputer& self, float theta, float phi, int ntime) {
    DefaultPointCloud cloud = self.compute_view(theta, phi);
    if (cloud.empty() && ntime < 10) {
            self.free_gpu();
            self.setGLFWContext();
            return compute_view_tweak_int(self, theta, phi, ntime + 1);
        }
    assert( !cloud.empty() );
    return cloud;
}
DefaultPointCloud compute_view_tweak(PartialViewComputer& self, float theta, float phi) {
    return compute_view_tweak_int(self, theta, phi, 0);
}


BOOST_PYTHON_MODULE(libpypartialview)
{
    class_<DefaultPointCloud>("PointCloud")
            .def( "size", &DefaultPointCloud::size )
            .add_property( "serialized_data", &serialize_pointCloud, &unserialize_pointcloud )
            .def( "save_pcd", &save_point_cloud_as_pcd)
            .def( "load_pcd", &read_point_cloud_from_pcd)
            .staticmethod( "load_pcd" );

    class_<PartialViewComputer>("PartialViewComputer")
            .def("display_mesh", &PartialViewComputer::displayMesh )
            .def("load_mesh", &PartialViewComputer::loadMesh )
            .def("compute_view", &compute_view_tweak );
}
