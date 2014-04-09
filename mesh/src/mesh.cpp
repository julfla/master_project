#include <string>

#include "mesh/mesh.hpp"

Mesh::Mesh(std::string path_file) {
    std::ifstream in_stream(path_file.c_str());
    if(!in_stream.good())
        throw std::ifstream::failure("Input file not found.");
    std::string line;
    while (std::getline(in_stream, line))
        polygons.push_back(TrianglePolygon(line));
    in_stream.close();
}

void Mesh::init_map_area() {
    double cumul = 0;
    for (int i=0; i < polygons.size(); ++i) {
        cumul += polygons.at(i).getArea();
        cumul_area[cumul] = polygons.at(i);
    }
    generator = boost::minstd_rand(std::time(0));
    uni_dist = boost::uniform_real<>(0,cumul);
}
