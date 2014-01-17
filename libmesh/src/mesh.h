#ifndef MESH_H
#define MESH_H

#include <iostream>
#include <fstream> // to read file
#include <vector>
#include <map>
#include <ctime>
#include <stdexcept> // to throw exceptions

#include "boost/random.hpp"

#include "triangle_polygon.h"
#include "point_3d.h"


class Mesh {

public:
	Mesh() {}
	Mesh(std::vector<TrianglePolygon> polygons) {
		this->polygons = polygons;
	}

    Mesh(std::string path_file);

	const Point_3D retreive_random_point() {
        if(cumul_area.empty())
            init_map_area();
		return cumul_area.upper_bound(uni_dist(generator))->second.getRandomPoint();
	}

	const bool empty() {
		return polygons.empty();
	}

    size_t size() {
        return polygons.size();
    }

private:
    std::vector<TrianglePolygon> polygons;
	//the map is used to retreive a random point on the mesh
    std::map<double, TrianglePolygon> cumul_area; //cumulative area of polygons

	//generator for random number
    boost::minstd_rand generator;
    boost::uniform_real<> uni_dist;
	
    void init_map_area();
};

#endif // MESH_H
