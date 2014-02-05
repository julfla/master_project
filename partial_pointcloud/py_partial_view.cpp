#include <boost/python.hpp>
#include <boost/python/wrapper.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <boost/assign/std/vector.hpp> // for 'operator+=()'

#include "partial_view.h"

using namespace boost::python;
using namespace boost::assign;

typedef std::vector<double> vec_dbl;

vec_dbl get_pointcloud_data_as_vector( DefaultPointCloud& self ) {
    vec_dbl buff;
    for( DefaultPointCloud::iterator it = self.points.begin(); it < self.points.end(); ++it )
        buff += it->x, it->y, it->z;
    return buff;
}

void set_pointcloud_data_from_vector( DefaultPointCloud& self, vec_dbl point_coordinates) {
    assert(point_coordinates.size() % 3 == 0); // x,y,z coordinates only
    for( vec_dbl::iterator it = point_coordinates.begin(); it < point_coordinates.end(); it = it+3)
        self.points += DefaultPoint(*it, *(it+1), *(it+2));
}

void save_point_cloud_as_pcd(DefaultPointCloud& self, char* path) {
    pcl::io::savePCDFileASCII(path, self);
}

BOOST_PYTHON_MODULE(libpypartialview)
{
    class_<vec_dbl>("ListDouble").def(vector_indexing_suite<vec_dbl>() );

    class_<DefaultPointCloud>("PointCloud")
            .add_property("data", &get_pointcloud_data_as_vector, &set_pointcloud_data_from_vector )
            .def("save_pcd", &save_point_cloud_as_pcd);

    class_<PartialViewComputer>("PartialViewComputer")
            .def("display_mesh", &PartialViewComputer::displayMesh )
            .def("load_mesh", &PartialViewComputer::loadMesh )
            .def("compute_view", &PartialViewComputer::compute_view );
}
