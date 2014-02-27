#include <boost/python.hpp>
#include <boost/python/wrapper.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <boost/assign/std/vector.hpp> // for 'operator+=() on vectors'

#include "partial_view.h"
#include "pointcloud_serialization.hpp"
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

using namespace boost::python;
using namespace boost::assign;

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

void save_point_cloud_as_pcd( DefaultPointCloud& self, char* path) { 
    pcl::io::savePCDFileASCII(path, self); 
}

/*
DefaultPointCloud read_point_cloud_from_pcd( std::string path) {
    DefaultPointCloud::Ptr cloud (new DefaultPointCloud);
    pcl::io::loadPCDFile<DefaultPoint> (path, *cloud);
    return *cloud;
}
*/

BOOST_PYTHON_MODULE(libpypartialview)
{
    class_<DefaultPointCloud>("PointCloud")
            .def( "size", &DefaultPointCloud::size )
            .add_property( "serialized_data", &serialize_pointCloud, &unserialize_pointcloud )
            .def( "save_pcd", &save_point_cloud_as_pcd);

    class_<PartialViewComputer>("PartialViewComputer")
            .def("display_mesh", &PartialViewComputer::displayMesh )
            .def("load_mesh", &PartialViewComputer::loadMesh )
            .def("compute_view", &PartialViewComputer::compute_view );
}
