#include "triangle_polygon.hpp"

boost::minstd_rand TrianglePolygon::generator(std::time(0));

std::ostream& operator<<(std::ostream &out, const TrianglePolygon &mesh) {
    for(int i = 0; i < 3; i++)
        out << mesh.points[i] << " ";
}
