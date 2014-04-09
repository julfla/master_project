#include <vector>

#define NPY_NO_DEPRECATED_API 8

#include <boost/python.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/python/numeric.hpp>
#include <numpy/ndarrayobject.h>

#include "descriptors/shape_distribution.h"


namespace bn = boost::python::numeric;

boost::python::object vectorToNumpyArray(std::vector<double> const& vec) { 
    npy_intp size = vec.size();
    double * data = size ? const_cast<double *>(&vec[0]) : static_cast<double *>(NULL);
    PyObject * pyObj = PyArray_SimpleNewFromData(1, &size, NPY_DOUBLE, data);
    boost::python::handle<> handle ( pyObj );
    boost::python::numeric::array arr(handle);
    return arr.copy();
}

std::string serialize( Distribution & self ) {
    std::ostringstream oss;
    boost::archive::text_oarchive archive( oss );
    archive << self;
    return oss.str();
}

void unserialize( Distribution & self, std::string data) {
    std::istringstream iss( data );
    boost::archive::text_iarchive archive( iss );
    archive >> self;
}

boost::python::object getNumpyArray(Distribution & self) { 
    return vectorToNumpyArray( self.getDistribution() );
}

/*****************
 Needs testing
******************
void setNumpyArray(Distribution & self, bn::array arr) {
    std::vector<double> buff;
    for (int i = 0; i < 10; ++i)
        buff.push_back(i);
    self.setDistribution(buff);
}
*/

BOOST_PYTHON_MODULE(libpydescriptors)
{
    using namespace boost::python;

    Py_Initialize();
    import_array();
    numeric::array::set_module_and_type("numpy", "ndarray");

    class_<Distribution>("Distribution")
            .def( init<pcl::PointCloud<pcl::PointXYZ> * const >() )
            .add_property( "serialized_data", &serialize, &unserialize )
            .add_property( "as_numpy_array", &getNumpyArray);
}
