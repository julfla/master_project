#ifndef PARTIAL_VIEW_HPP
#define PARTIAL_VIEW_HPP

//should be defined by cmake... ?
#define DEBUG true

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

#include <mesh.h>
#include "shader_helper.hpp"

#ifdef DEBUG
#define DEBUG_MSG(str) do { cout << str << std::endl; } while( false )
#else
#define DEBUG_MSG(str) do { } while ( false )
#endif

using namespace glm;

class PartialViewComputer {

public:

    void loadMesh(std::string path);

    bool presets();

    void init_MVP(float theta, float phi);

    pcl::PointCloud<pcl::PointXYZ> compute_view(float theta, float phi, bool show_image);

    void draw();

    void displayMesh();

    PartialViewComputer(std::string tri_path, int width = 800, int height = 600, float fov = 45.0f) {
        this->width = width;
        this->height = height;
        this->fov = fov;
        this->windowsLessContextSet = false;
        this->glfwContextSet = false;
        loadMesh(tri_path);
    }

    ~PartialViewComputer() {
        free_gpu();
    }

private:

    void build_cloud_from_framebuffer(pcl::PointCloud<pcl::PointXYZ> * cloud);

    void build_cloud_from_pixelbuffer(pcl::PointCloud<pcl::PointXYZ> * cloud);

    void free_gpu() {
        // Cleanup VBO and shader
        glDeleteBuffers(1, &vertexbuffer);
        glDeleteProgram(programID);
        DEBUG_MSG( "GPU Freed." );
        // Close OpenGL window and terminate GLFW
        if(glfwContextSet)
            glfwTerminate();
        windowsLessContextSet = glfwContextSet = false;
    }

    bool setGLFWContext(const char* window_name = NULL);

    // Create an windowless opengl context
    // credit : http://renderingpipeline.com/2012/05/windowless-opengl/
    bool setWindowlessContext();

    bool windowsLessContextSet, glfwContextSet;
    float max_relative_position[3]; // used to remove the border effect..
    float min_relative_position[3]; //
    std::vector<glm::vec3> g_vertex_buffer_data; // vertice of the model
    std::vector<glm::vec3> outter_box; // a cubic box containing all the model
    glm::vec3 centroid; //centroid of the model
    float scale_factor;
    int width, height;
    float fov;
    glm::mat4 SCALING, MVP;
    GLuint programID, MVP_ID, MV_ID, SCALE_ID, vertexPosition_modelspaceID, vertexbuffer;//, framebuffer;



};

#endif //PARTIAL_VIEW_HPP

