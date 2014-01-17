#include"vector_3d.h"

Vector_3D operator*(double scalar, const Vector_3D& vect) {
    return vect * scalar;
}

std::ostream & operator<<(std::ostream &out, const Vector_3D &vect) {
    out << "x:" << vect.x;
    out << " y:" << vect.y;
    out << " z:" << vect.z;
}



