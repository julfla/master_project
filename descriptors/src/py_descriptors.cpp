#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include "shape_distribution.h"

inline Distribution load_archive_char(char* path) { return Distribution::load_archive(path); }

BOOST_PYTHON_MODULE(libpydescriptors)
{
    using namespace boost::python;

    typedef std::vector<double> vec_dbl;

    class_<vec_dbl>("ListDouble")
           .def(vector_indexing_suite<vec_dbl>() );

    class_<Distribution>("ShapeDistribution", no_init)
            .add_property("data", &Distribution::getDistribution)
            .def("load_archive", load_archive_char )
            .staticmethod("load_archive")
            .def("save_archive", (void (Distribution::*)(const char*) ) &Distribution::save_archive);
}
