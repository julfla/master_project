#ifndef POINT_3D_H
#define POINT_3D_H

#include <iostream>
#include <cmath>  // for fabs (float_abs)

class Point_3D {
    friend std::ostream & operator<<(std::ostream & out, const Point_3D &f);
    friend Point_3D operator*(double scalar, const Point_3D &point);

public:
    Point_3D() : x(0), y(0), z(0) {}
    Point_3D(double x, double y, double z) : x(x), y(y), z(z) {}
    
    double getX() const {return x;}
    double getY() const {return y;}
    double getZ() const {return z;}
    void setX(double x) {this->x = x;}
    void setY(double y) {this->y = y;}
    void setZ(double z) {this->z = z;}

    const double distance(Point_3D point) {
        double sum = pow(x - point.getX(), 2);
        sum += pow(y - point.getY(), 2);
        sum += pow(z - point.getZ(), 2);
        return sqrt(sum);
    }

    Point_3D operator*(const double scalar) const {
        return Point_3D(x*scalar, y*scalar, z*scalar);
    }

    Point_3D operator+(const Point_3D& point) const {
        return Point_3D(x+point.getX(), y+point.getY(),
                        z+point.getZ());
    }

    Point_3D operator-(const Point_3D& point) const {
        return Point_3D(x-point.getX(), y-point.getY(),
                        z-point.getZ());
    }

    const bool operator==(const Point_3D& point) const {
        return fabs(x-point.getX()) + fabs(y-point.getY())
                + fabs(z-point.getZ()) < 1e-7;
    }

private:
    double x, y, z;

};

#endif  // POINT_3D_H
