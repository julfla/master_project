#ifndef VECTOR_3D_H
#define VECTOR_3D_H

#include "point_3d.h"
#include <cmath> // for abs
#include <boost/random.hpp> // for get random point

class Vector_3D {

  friend std::ostream & operator<<(std::ostream & out, const Vector_3D &vect);
  friend Vector_3D operator*(double scalar, const Vector_3D &vect);
public:
  Vector_3D() : x(0), y(0), z(0)
  {}
  Vector_3D(double x, double y, double z) : x(x), y(y), z(z)
  {}
  Vector_3D(const Point_3D& tail, const Point_3D& head)
  {
    x = head.getX() - tail.getX();
    y = head.getY() - tail.getY();
    z = head.getZ() - tail.getZ();
  }
  
  double getX() const {return x;}
  double getY() const {return y;} 
  double getZ() const {return z;}    
  
  void setX(double x) {this->x = x;}
  void setY(double y) {this->y = y;}
  void setZ(double z) {this->z = z;}
  
  bool operator==(const Vector_3D& vect) const
  {
    return abs(x-vect.x) + abs(y-vect.y) + abs(z-vect.z) < 1e-7;
  }
  
  Vector_3D operator+(const Vector_3D &vect) const{
    return Vector_3D(x+vect.getX(), y+vect.getY(), z+vect.getZ());
  }
  
  Vector_3D operator-(const Vector_3D &vect) const{
    return Vector_3D(x-vect.getX(), y-vect.getY(), z-vect.getZ());
  }
  
  Vector_3D operator*(const double scalar) const{
    return Vector_3D(x*scalar,y*scalar,z*scalar);
  }
  
  //The operator * will be used as dot product operator
  double operator*(const Vector_3D &vect) const{
    return x*vect.getX() + y*vect.getY() + z*vect.getZ();
  }
  
   //The operator % will be used as cross product operator
  Vector_3D operator%(const Vector_3D &vect) const{
    return Vector_3D(y*vect.getZ() - z*vect.getY(),   
		     x*vect.getZ() - z*vect.getX(),
		     x*vect.getY() - y*vect.getX());
  }
  
  double norm() {return sqrt(*this * *this);}
  
private:
  double x,y,z;
  
};


#endif // VECTOR_3D_H
