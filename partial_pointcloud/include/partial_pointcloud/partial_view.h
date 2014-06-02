#ifndef PARTIAL_VIEW_HPP
#define PARTIAL_VIEW_HPP

#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#define GLM_FORCE_RADIANS // all angles are in radians 
#include <glm/glm.hpp>
#include <GLFW/glfw3.h>

#include "mesh/mesh.hpp"

typedef pcl::PointXYZ DefaultPoint;
typedef pcl::PointCloud<DefaultPoint> DefaultPointCloud;

class PartialViewComputer {

public:

    void loadMesh(std::string mesh_path);
    //void loadMesh(Mesh & mesh);

    DefaultPointCloud compute_view(float theta, float phi);

    int displayMesh(float theta, float phi);

    PartialViewComputer(int width = 800, int height = 600, float fov = M_PI / 4) :
        width(width), height(height), fov(fov) {init();}

    PartialViewComputer(std::string tri_path, int width = 800, int height = 600, float fov = M_PI / 4) :
        width(width), height(height), fov(fov) {init(); loadMesh(tri_path);}

    ~PartialViewComputer();

    void show_window() {glfwShowWindow(window);}

    void hide_window() {glfwHideWindow(window);}

private:

    bool init();

    void draw();

    glm::mat4 view_matrix(float theta, float phi);

    // Returns a a width*height long vector containing raw data after drawing.
    const std::vector<glm::vec4> framebuffer_data();

    void init_MVP(float theta, float phi);

    std::vector<glm::vec3> vertice; // vertice of the model
    glm::vec3 centroid; //centroid of the model
    float scale_factor; // scaling needed to fit the mesh in a 1 radius sphere
    int width, height;
    float fov;
    GLFWwindow* window;
    glm::mat4 M,V,P;
    GLuint programID, mID, vID, pID, vertexbuffer, VertexArrayID;
};

#endif //PARTIAL_VIEW_HPP

