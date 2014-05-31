// Include standard headers
#include <string>
#include <vector>
#include <iostream>
#include <stdlib.h>

// Include GLEW
#include <GL/glew.h>

// Include GLFW
#include <GLFW/glfw3.h>

// Include GLM
#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtx/norm.hpp> // provides glm::length2 for euclidian norm of glm::vec

#include "partial_pointcloud/shader_helper.hpp"
#include "partial_pointcloud/partial_view.h"

#include "debug_helper.hpp" // provides DEBUG_MSG macro

void PartialViewComputer::loadMesh(std::string mesh_path) {
    DEBUG_MSG( "Loading mesh.");
    Mesh mesh(mesh_path);
    //add the mesh triangles to the vertice list
    vertice.clear();
    for ( std::vector<TrianglePolygon>::const_iterator poly = mesh.get_polygons().begin(); 
        poly < mesh.get_polygons().end(); ++poly) {
            for ( int i = 0; i < 3; ++i) {
                Point_3D pts = poly->getPoints()[i];
                glm::vec3 vertex(pts.getX(), pts.getY(), pts.getZ());
                vertice.push_back( vertex );
            }
    }
  centroid = glm::vec3(0, 0, 0);
    for (std::vector<glm::vec3>::iterator it = vertice.begin();
        it < vertice.end(); ++it) {
        centroid += *it;
    }
    centroid /= vertice.size();    
    //scale_factor to use to fit the object in a 1 radius sphere
    scale_factor = 0.0f;
    for (std::vector<glm::vec3>::iterator it = vertice.begin();
        it < vertice.end(); ++it) {
        float dist = glm::l2Norm( *it, centroid);
        if( scale_factor < dist)
            scale_factor = dist;
    }
    //bind the vertice to the vertexbuffer
    glDeleteBuffers(1, &vertexbuffer);
    glGenBuffers(1, &vertexbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
    glBufferData(GL_ARRAY_BUFFER, vertice.size() * sizeof(glm::vec3), &vertice[0], GL_STATIC_DRAW);
    DEBUG_MSG( vertice.size() << " triangles loaded." );
    DEBUG_MSG( "Centroid : " << centroid.x << " " << centroid.y << " " << centroid.z );
    DEBUG_MSG( "Scale factor : " << scale_factor );
}

bool PartialViewComputer::init() {
    DEBUG_MSG( "Initializing...");
    // Initialise GLFW
    if( !glfwInit() )
    {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return false;
    }

    glfwWindowHint(GLFW_SAMPLES, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // Open a window and create its OpenGL context
    window = glfwCreateWindow( 1024, 768, "OpenGL", NULL, NULL );
    if( window == NULL ){
        std::cerr << "Failed to open GLFW window. " << 
            "If you have an Intel GPU, they are not 3.3 compatible." << std::endl;
        return false;
    }
    glfwMakeContextCurrent(window);

    // Initialize GLEW
    glewExperimental = true; // Needed for core profile
    if (glewInit() != GLEW_OK) {
        std::cerr << "Failed to initialize GLEW" << std::endl;
        return false;
    }

    // Ensure we can capture the escape key being pressed below
    glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);

    // White background, 4th coordinate to 0 for background segmentation
    glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
    // Enable depth test
    glEnable(GL_DEPTH_TEST); glDepthFunc(GL_LESS); 

    glGenVertexArrays(1, &VertexArrayID);
    glBindVertexArray(VertexArrayID);

    // Create and compile our GLSL program from the shaders
    programID = LoadShaders( "../shaders/TransformVertexShader.vertexshader", "../shaders/ColorFragmentShader.fragmentshader" );
    mID = glGetUniformLocation(programID, "M");
    vID = glGetUniformLocation(programID, "V");
    pID = glGetUniformLocation(programID, "P");
    DEBUG_MSG( "Initialization finished");

}

DefaultPointCloud PartialViewComputer::compute_view(float theta, float phi) {
    init_MVP(theta, phi);
    std::vector<glm::vec4> data = framebuffer_data();
    DefaultPointCloud cloud;
    for(std::vector<glm::vec4>::iterator it = data.begin(); it < data.end(); ++it) {
        // Use the w coordinate for segmenting background
        bool isBackground = ( it->w <= 1.0f - 1.0e-7);
        if( !isBackground )
            cloud.push_back( DefaultPoint(it->x,it->y,it->z) );
    }
    DEBUG_MSG( "data length : " << data.size() );
    DEBUG_MSG( "cloud size : " << cloud.size() );
    if (cloud.empty()) { // print additional debug info
        for(std::vector<glm::vec4>::iterator it = data.begin(); it < data.end(); ++it) {
            // DEBUG_MSG( "x:" << it->x << "  y:" << it->y << "  z:" << it->z << "  w:" << it->w );
        }
    }
    return cloud;
}

int PartialViewComputer::displayMesh(float theta, float phi) {
    init_MVP(theta, phi);
    do{
        draw();
        glfwSwapBuffers(window); // Swap buffers
        glfwPollEvents(); // Deal with events on the window

    } // Stop if the ESC key was pressed or the window was closed
    while( glfwGetKey(window, GLFW_KEY_ESCAPE ) != GLFW_PRESS &&
           glfwWindowShouldClose(window) == 0 );

    return 0;
}

const std::vector<glm::vec4> PartialViewComputer::framebuffer_data() {
    // switch rendering to a framebuffer
    
    //TODO : we need to draw inside the window once before...
    // we have to fix this
    draw();
    glfwSwapBuffers(window); // Swap buffers
    
    GLuint fbo;
    glGenFramebuffers(1,&fbo);
    glFramebufferRenderbuffer(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, vertexbuffer);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER,fbo);

    draw();

    // read data from the fbo
    std::vector<glm::vec4> data(width*height);
    glReadBuffer(GL_COLOR_ATTACHMENT0);
    glReadPixels(0,0,width,height,GL_RGBA,GL_FLOAT,&data[0]);

    // Return to onscreen rendering:
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER,0);
    glDeleteFramebuffers(1,&fbo);
    
    return data;
}

void PartialViewComputer::init_MVP(float theta, float phi) {
    P = glm::perspective( (float) M_PI / 4 /* 45° */, 4.0f / 3.0f, 0.1f, 100.0f);
    V = view_matrix(theta, phi);
    // M center the model on (0,0,0) and scale it to fit in a 1 radius sphere
    M = glm::scale(glm::vec3(1.0f/scale_factor)) * glm::translate(-centroid);
}

glm::mat4 PartialViewComputer::view_matrix(float theta, float phi) {
    // min distance to see the full shere given the angle of view fov
    float r = 1.0 / std::tan(fov/2);
    r *= 1.1f; // 10% margin
    // coodinate on a r radius sphere
    float x = r * std::sin(theta) * std::cos(phi);
    float y = r * std::sin(theta) * std::sin(phi);
    float z = r * std::cos(theta);
    
    DEBUG_MSG( "Viewing distance :" << r);
    DEBUG_MSG( "Cam Info :" << x << " " << y << " " << z );
    return glm::lookAt(
                glm::vec3(x,y,z), // Coordinates of the camera, in World Space
                glm::vec3(0,0,0), // and looks at there in world space
                glm::vec3(0,1,0) // Head is up (set to 0,-1,0 to look upside-down)
                );
}

void PartialViewComputer::draw() {
    // Clear the screen
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glUseProgram(programID);
    glUniformMatrix4fv(mID, 1, GL_FALSE, &M[0][0]);
    glUniformMatrix4fv(vID, 1, GL_FALSE, &V[0][0]);
    glUniformMatrix4fv(pID, 1, GL_FALSE, &P[0][0]);

    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
    glVertexAttribPointer(
        0,                  // attribute 0. No particular reason for 0, but must match the layout in the shader.
        3,                  // size
        GL_FLOAT,           // type
        GL_FALSE,           // normalized?
        0,                  // stride
        (void*)0            // array buffer offset
    );
    // Draw the triangles !
    glDrawArrays(GL_TRIANGLES, 0, vertice.size());
    glDisableVertexAttribArray(0);
}

PartialViewComputer::~PartialViewComputer() {
    // Cleanup VBO
    glDeleteBuffers(1, &vertexbuffer);
    glDeleteVertexArrays(1, &VertexArrayID);
    glDeleteProgram(programID);

    // Close OpenGL window and terminate GLFW
    glfwTerminate();
}
