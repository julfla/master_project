#ifndef PARTIAL_VIEW_HPP
#define PARTIAL_VIEW_HPP

// Include standard headers
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <cmath>
#include <inttypes.h>
#include <limits>
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

// Include GLEW
#include <GL/glew.h>

// Include GLFW
#include <GL/glfw.h>

// Include GLM
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/transform.hpp>

// Include GLX
#include <X11/X.h>
#include <X11/Xlib.h>
#include <GL/gl.h>
#include <GL/glx.h>

#include "mesh/mesh.hpp"
#include "partial_pointcloud/shader_helper.hpp"

typedef pcl::PointXYZ DefaultPoint;
typedef pcl::PointCloud<DefaultPoint> DefaultPointCloud;

using namespace glm;

class PartialViewComputer {

public:

    void loadMesh(std::string path);

    DefaultPointCloud compute_view(float theta, float phi);

    void displayMesh(float theta, float phi);

    PartialViewComputer(int width = 800, int height = 600, float fov = 45.0f) :
        width(width), height(height), fov(fov), windowsLessContextSet(false), glfwContextSet(false) {}

    PartialViewComputer(std::string tri_path, int width = 800, int height = 600, float fov = 45.0f) :
        width(width), height(height), fov(fov), windowsLessContextSet(false), glfwContextSet(false) {loadMesh(tri_path);}

    ~PartialViewComputer() {free_gpu();}

    // should be private but exposed for stability tweak
    void free_gpu();

    // should be private but exposed for stability tweak
    bool setGLFWContext(const char* window_name = NULL);
private:

    bool presets();

    glm::mat4 ViewMatrix(float theta, float phi);

    glm::mat4 ProjectionMatrix(float theta, float phi);

    void init_MVP(float theta, float phi);

    void draw();

    void pixel_vector_to_pointcloud(std::vector<float> * data, DefaultPointCloud * cloud);

    void build_cloud_from_framebuffer(DefaultPointCloud * cloud);


    // Create an windowless opengl context
    // credit : http://renderingpipeline.com/2012/05/windowless-opengl/
    bool setWindowlessContext();

    bool windowsLessContextSet, glfwContextSet;
    float max_relative_position[3]; // used to remove the border effect..
    float min_relative_position[3]; //
    std::vector<float> g_vertex_buffer_data; // vertice of the model
    double containing_diameter;
    glm::vec3 centroid; //centroid of the model
    float scale_factor;
    int width, height;
    float fov;
    glm::mat4 SCALING, MVP;
    GLuint programID, MVP_ID, MV_ID, SCALE_ID, vertexPosition_modelspaceID, vertexbuffer;//, framebuffer;

};

#endif //PARTIAL_VIEW_HPP

