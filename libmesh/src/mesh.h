#ifndef MESH_H
#define MESH_H

#include "triangle_polygon.h"
#include "point_3d.h"
#include "boost/random.hpp"
#include <vector>
#include <map>
#include <ctime>

#include <stdexcept> // to throw exceptions
#include <fstream> // to read file
#include <iostream>

class Mesh {

public:
	Mesh() {}
	Mesh(std::vector<TrianglePolygon> polygons) {
		this->polygons = polygons;
	}

    Mesh(std::string path_file) {
        std::ifstream in_stream(path_file.c_str());
        if(!in_stream.good())
            throw std::ifstream::failure("Input file not found.");
        std::string line;
        while (std::getline(in_stream, line))
            polygons.push_back(TrianglePolygon(line));
        in_stream.close();
    }

	const Point_3D retreive_random_point() {
        if(cumul_area.empty())
            init_map_area();
		return cumul_area.upper_bound(uni_dist(generator))->second.getRandomPoint();
	}

	const bool empty() {
		return polygons.empty();
	}

private:
    std::vector<TrianglePolygon> polygons;
	//the map is used to retreive a random point on the mesh
	std::map<double, TrianglePolygon> cumul_area; //cumlative area of polygons

	//generator for random number
    boost::minstd_rand generator;
    boost::uniform_real<> uni_dist;
	
	void init_map_area() {
        double cumul = 0;
        for (int i=0; i < polygons.size(); ++i) {
            cumul += polygons.at(i).getArea();
            cumul_area[cumul] = polygons.at(i);
        }
        generator = boost::minstd_rand(std::time(0));
        uni_dist = boost::uniform_real<>(0,cumul);
	}
};

#endif // MESH_H
