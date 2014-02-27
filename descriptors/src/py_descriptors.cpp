#include <boost/python.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

#include "shape_distribution.h"

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

BOOST_PYTHON_MODULE(libpydescriptors)
{
    using namespace boost::python;

    class_<Distribution>("ShapeDistribution", no_init)
            .add_property( "serialized_data", &serialize, &unserialize );
}
