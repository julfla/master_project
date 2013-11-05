// Include standard headers
#include <stdio.h>
#include <stdlib.h>

// Include GLEW
#include <GL/glew.h>

// Include GLFW
#include <GL/glfw.h>

// Include GLM
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/transform.hpp>
using namespace glm;

#include <mesh.h>

#include "common/shader.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <vector>
#include <cmath>
#include <inttypes.h>

#include <limits>

bool init_view(int width, int height) {
    // Initialise GLFW
    if( !glfwInit() )
    {
        fprintf( stderr, "Failed to initialize GLFW\n" );
        return false;
    }

    glfwOpenWindowHint(GLFW_FSAA_SAMPLES, 4);
    glfwOpenWindowHint(GLFW_OPENGL_VERSION_MAJOR, 2);
    glfwOpenWindowHint(GLFW_OPENGL_VERSION_MINOR, 1);

    // Open a window and create its OpenGL context
    if( !glfwOpenWindow( width, height, 0,0,0,0, 32,0, GLFW_WINDOW ) )
    {
        fprintf( stderr, "Failed to open GLFW window.\n" );
        glfwTerminate();
        return -1;
    }

    // Initialize GLEW
    if (glewInit() != GLEW_OK) {
        fprintf(stderr, "Failed to initialize GLEW\n");
        return -1;
    }

    glfwSetWindowTitle( "Tutorial 04" );

    // Ensure we can capture the escape key being pressed below
    glfwEnable( GLFW_STICKY_KEYS );

    // Dark blue background
    glClearColor(1.0f, 1.0f, 1.0f, 0.0f);

    return true;

}

void read_from_file(std::string path, std::vector<glm::vec3> & vertice) {
    std::ifstream in_stream(path.c_str());
    if(!in_stream.good())
        throw std::ifstream::failure("Input file not found.");
    std::string line;
    while (std::getline(in_stream, line)) {
        TrianglePolygon tri(line);
        for(int i = 0; i < 3; ++i) {
            glm::vec3 vertex;
            vertex.x = (float) tri.getPoints()[i].getX();
            vertex.y = (float) tri.getPoints()[i].getY();
            vertex.z = (float) tri.getPoints()[i].getZ();
            vertice.push_back(vertex);
        }
    }
    in_stream.close();
}

int main()
{
    int width = 800;
    int height = 600;
    float FOV = 45.0f;
    std::string tri_path = "bottle1.tri";

    std::vector<glm::vec3> g_vertex_buffer_data;
    read_from_file(tri_path, g_vertex_buffer_data);
    std::cout << g_vertex_buffer_data.size();
    std::vector<glm::vec3> outter_box;
    glm::vec3 centroid;
    float scale_factor;
    {
        float x_max, y_max, z_max;
        x_max = y_max = z_max = - std::numeric_limits<float>::max();
        float x_min, y_min, z_min;
        x_min = y_min = z_min = std::numeric_limits<float>::max();
        float x_mean, y_mean, z_mean;
        x_mean = y_mean = z_mean = 0;
        for(std::vector<glm::vec3>::iterator it = g_vertex_buffer_data.begin(); it < g_vertex_buffer_data.end(); ++it) {
            if(it->x > x_max) x_max = it->x;
            if(it->x < x_min) x_min = it->x;
            if(it->y > y_max) y_max = it->y;
            if(it->y < y_min) y_min = it->y;
            if(it->z > z_max) z_max = it->z;
            if(it->z < z_min) z_min = it->z;
        }

        x_mean = ( x_max + x_min) / 2;
        y_mean = ( y_max + y_min) / 2;
        z_mean = ( z_max + z_min) / 2;
        centroid = glm::vec3(x_mean, y_mean, z_mean);

        scale_factor = x_max;
        scale_factor = max(scale_factor, - x_min);
        scale_factor = max(scale_factor, y_min);
        scale_factor = max(scale_factor, - y_min);
        scale_factor = max(scale_factor, z_max);
        scale_factor = max(scale_factor, - z_min);
        scale_factor = 1.0 / scale_factor;


        outter_box.push_back(glm::vec3(x_max - x_mean, y_max - y_mean, z_max - z_mean));
        outter_box.push_back(glm::vec3(x_max - x_mean, y_max - y_mean, z_min - z_mean));
        outter_box.push_back(glm::vec3(x_max - x_mean, y_min - y_mean, z_max - z_mean));
        outter_box.push_back(glm::vec3(x_max - x_mean, y_min - y_mean, z_min - z_mean));
        outter_box.push_back(glm::vec3(x_min - x_mean, y_max - y_mean, z_max - z_mean));
        outter_box.push_back(glm::vec3(x_min - x_mean, y_max - y_mean, z_min - z_mean));
        outter_box.push_back(glm::vec3(x_min - x_mean, y_min - y_mean, z_max - z_mean));
        outter_box.push_back(glm::vec3(x_min - x_mean, y_min - y_mean, z_min - z_mean));

        std::cout << x_min << " < x < " << x_max << " average : " << x_mean << std::endl;
        std::cout << y_min << " < y < " << y_max << " average : " << y_mean << std::endl;
        std::cout << z_min << " < z < " << z_max << " average : " << z_mean << std::endl;
        std::cout << "Scale factor : " << scale_factor << " inv: " << 1.0f / scale_factor << std::endl;

    }

    double theta = M_PI * 0.30;
    double phi = 2 * M_PI * 0.125;

    // Model matrix : translate model centroid to origin and scale it
    glm::mat4 Model   = glm::translate(- centroid);

    // Camera matrix

    //coodinate on 1radius sphere
    glm::vec3 cam_pos(std::sin(theta) * std::cos(phi), std::sin(theta) * std::sin(phi), std::cos(theta));

    //find the distance min
    float view_dist;
    for(std::vector<glm::vec3>::iterator it = outter_box.begin(); it < outter_box.end(); ++it) {
        float axial_proj = glm::dot(cam_pos, *it);
        float current_radial_dist = std::sqrt( glm::dot(*it,*it) - axial_proj * axial_proj);
        float current_view_dist = current_radial_dist / std::tan(M_PI * FOV / 180.0f) - axial_proj;

        if( current_view_dist > view_dist)
            view_dist = current_view_dist;
    }
    //view_dist *= scale_factor;
    //scale the view position
    cam_pos.x *= 1.5 * view_dist;
    cam_pos.y *= 1.5 * view_dist;
    cam_pos.z *= 1.5 * view_dist;

    std::cout << "Cam Info :" << std::endl;
    std::cout << "     Pos : " << cam_pos.x << " " << cam_pos.y << " " << cam_pos.z << std::endl;

    glm::mat4 View       = glm::lookAt(
                cam_pos, // Camera is at (4,3,-3), in World Space
                glm::vec3(0,0,0), // and looks at the origin
                glm::vec3(0,1.0,0.0) // Head is up (set to 0,-1,0 to look upside-down)
                );


    // Projection matrix : FOV Field of View, 4:3 ratio, display range : 0.1 unit <-> 100 units
    glm::mat4 Projection = glm::perspective(FOV, 4.0f / 3.0f, 0.1f * view_dist, 10.0f * view_dist);

    // Our ModelViewProjection : multiplication of our 3 matrices
    glm::mat4 MVP        = Projection * View * Model;
    glm::mat4 MV         = View * Model;

    if(!init_view(width, height)) return -1;

    // Enable depth test
    glEnable(GL_DEPTH_TEST);
    // Accept fragment if it closer to the camera than the former one
    glDepthFunc(GL_LESS);

    // Create and compile our GLSL program from the shaders
    GLuint programID = LoadShaders( "shaders/TransformVertexShader.vertexshader", "shaders/ColorFragmentShader.fragmentshader" );

    // Get a handle for our "MVP" uniform
    GLuint MVP_ID = glGetUniformLocation(programID, "MVP");
    GLuint SCALE_ID = glGetUniformLocation(programID, "SCALE");
    GLuint MV_ID = glGetUniformLocation(programID, "MV");

    // Get a handle for our buffers
    GLuint vertexPosition_modelspaceID = glGetAttribLocation(programID, "vertexPosition_modelspace");
    GLuint vertexColorID = glGetAttribLocation(programID, "vertexColor");

    GLuint vertexbuffer;
    glGenBuffers(1, &vertexbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
    glBufferData(GL_ARRAY_BUFFER, g_vertex_buffer_data.size() * sizeof(glm::vec3), &g_vertex_buffer_data[0], GL_STATIC_DRAW);

    std::vector<float> data(width*height*3);
    do{

        // Clear the screen
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Use our shader
        glUseProgram(programID);

        glUniformMatrix4fv(MVP_ID, 1, GL_FALSE, &MVP[0][0]);
        glUniformMatrix4fv(MV_ID, 1, GL_FALSE, &MV[0][0]);
        glUniform1f(SCALE_ID, scale_factor);

        // 1rst attribute buffer : vertices
        glEnableVertexAttribArray(vertexPosition_modelspaceID);
        glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
        glVertexAttribPointer(
                    vertexPosition_modelspaceID, // The attribute we want to configure
                    3,                           // size
                    GL_FLOAT,                    // type
                    GL_FALSE,                    // normalized?
                    0,                           // stride
                    (void*)0                     // array buffer offset
                    );

        // Draw the triangleS !
        glDrawArrays(GL_TRIANGLES, 0, g_vertex_buffer_data.size()*3); // 12*3 indices starting at 0 -> 12 triangles

        glDisableVertexAttribArray(vertexPosition_modelspaceID);
        glDisableVertexAttribArray(vertexColorID);

        glReadBuffer(GL_BACK);
        glReadPixels(0,0,width,height,GL_RGB,GL_FLOAT,&data[0]);

        // Swap buffers
        glfwSwapBuffers();

    } // Check if the ESC key was pressed or the window was closed
    while( glfwGetKey( GLFW_KEY_ESC ) != GLFW_PRESS &&
           glfwGetWindowParam( GLFW_OPENED ) );

    // Close OpenGL window and terminate GLFW
    glfwTerminate();


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>());
    for(std::vector<float>::iterator it = data.begin(); it < data.end(); ++it) {
        float x = *it;
        ++it;
        float y = *it;
        ++it;
        float z = *it;
        if( abs(x - 1.0) > 1e-5 && abs(y - 1.0) > 1e-5 && abs(z - 1.0) > 1e-5)
            cloud->push_back(pcl::PointXYZ(x,y,z));
    }

    std::cout << width * height << " pixels." << std::endl;
    std::cout << cloud->size() << " points in cloud." << std::endl;

    if(!cloud->empty())
        pcl::io::savePCDFileASCII ("test_pcd.pcd", *cloud);

    // Cleanup VBO and shader
    glDeleteBuffers(1, &vertexbuffer);
    glDeleteProgram(programID);

    return 0;
}
