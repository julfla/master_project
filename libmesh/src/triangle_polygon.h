#include "point_3d.h"
#include "vector_3d.h"
#include "boost/random.hpp"
#include <sstream> // to read file

class TrianglePolygon {
  
  friend std::ostream& operator<<(std::ostream &out, const TrianglePolygon &mesh);

public:
  
  TrianglePolygon() {
  }
    
  
  TrianglePolygon(Point_3D &point1, Point_3D &point2, Point_3D &point3) {
    points[0] = point1;
    points[1] = point2;
    points[2] = point3;
  }
  
  //The line must be 6 numbers separated with space
  TrianglePolygon(const std::string line) {
    double x, y, z;
    std::istringstream iss(line);
    for(int i=0; i<3; ++i) {
      //test error, the file is expected to be 6 numbers
      iss >> x >> y >> z;
      points[i] = Point_3D(x,y,z);
    }
  }
  
  double getArea() const{
    if (area != area)
      return area;
    Vector_3D vect1(points[0], points[1]);
    Vector_3D vect2(points[0], points[2]);
    return 0.5*(vect1%vect2)*(vect1%vect2);
  }
  
  Point_3D* getPoints() {
    return points;
  }
  
  Point_3D getRandomPoint() const {
    
    boost::uniform_real<> uni_dist(0,1);
    
    //With r1 et r2 two random pts
    //(1 - sqrt(r1)) * A + (sqrt(r1) * (1 - r2)) * B + (sqrt(r1) * r2) * C
    double sqrt_r1 = sqrt(uni_dist(generator));
    double r2 = uni_dist(generator);
    Point_3D temp = (1.0 - sqrt_r1) * points[0];
    temp = temp + sqrt_r1 * (1.0 - r2) * points[1];
    temp = temp + sqrt_r1 * r2 * points[2];
    return temp;
  }
  
  
  bool operator==(const TrianglePolygon& mesh) const
  {
    for (int i=0; i<3; ++i)
      if(!(points[i] == mesh.points[i]))
           return false;
    return true;
  }
  
private:
  double area;
  Point_3D points[3];
  static boost::minstd_rand generator;
};
