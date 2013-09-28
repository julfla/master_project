#include "point_3d.h"

  std::ostream & operator<<(std::ostream &out, const Point_3D &point)
  {
    out << "x:" << point.x;
    out << " y:" << point.y;
    out << " z:" << point.z;
  }
  
  Point_3D operator*(double scalar, const Point_3D &point)
  {
    return point*scalar;
  }

