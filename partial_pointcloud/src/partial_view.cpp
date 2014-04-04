#include "partial_pointcloud/partial_view.h"
#include <glm/gtx/norm.hpp> // prived glm::length2 for euclidian norm of glm::vec
#include "debug_helper.hpp"

void PartialViewComputer::loadMesh(std::string path) {
    //parse the file
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
            g_vertex_buffer_data.push_back(vertex);
        }
    }
    in_stream.close();

    //comptute the containing box and the centroid

    float x_max, y_max, z_max;
    x_max = y_max = z_max = - std::numeric_limits<float>::max();
    float x_min, y_min, z_min;
    x_min = y_min = z_min = std::numeric_limits<float>::max();
    float x_mean, y_mean, z_mean;

    // TODO : remove and do while parsing file !!
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

    //scale_factor will be used to make sure that pixels coordinates belong to [0,1]
    scale_factor = x_max - x_min;
    scale_factor = std::max(scale_factor, y_max - y_min);
    scale_factor = std::max(scale_factor, z_max - z_min);
    scale_factor = 1.0f / scale_factor;

    max_relative_position[0] = x_max * scale_factor;
    max_relative_position[1] = y_max * scale_factor;
    max_relative_position[2] = z_max * scale_factor;

    min_relative_position[0] = x_min * scale_factor;
    min_relative_position[1] = y_min * scale_factor;
    min_relative_position[2] = z_min * scale_factor;

    std::vector<glm::vec3> outter_box; // a cubic box containing all the model
    outter_box.push_back(glm::vec3(x_max - x_mean, y_max - y_mean, z_max - z_mean));
    outter_box.push_back(glm::vec3(x_max - x_mean, y_max - y_mean, z_min - z_mean));
    outter_box.push_back(glm::vec3(x_max - x_mean, y_min - y_mean, z_max - z_mean));
    outter_box.push_back(glm::vec3(x_max - x_mean, y_min - y_mean, z_min - z_mean));
    outter_box.push_back(glm::vec3(x_min - x_mean, y_max - y_mean, z_max - z_mean));
    outter_box.push_back(glm::vec3(x_min - x_mean, y_max - y_mean, z_min - z_mean));
    outter_box.push_back(glm::vec3(x_min - x_mean, y_min - y_mean, z_max - z_mean));
    outter_box.push_back(glm::vec3(x_min - x_mean, y_min - y_mean, z_min - z_mean));

    // compute diameter of containing sphere
    containing_diameter = 0;
    for (std::vector<glm::vec3>::iterator it = outter_box.begin(); it < outter_box.end(); ++it) {
        if( containing_diameter < glm::length2(*it) )
            containing_diameter = glm::length(*it);
    }

    DEBUG_MSG( x_min << " < x < " << x_max << " average : " << x_mean );
    DEBUG_MSG( y_min << " < y < " << y_max << " average : " << y_mean );
    DEBUG_MSG( z_min << " < z < " << z_max << " average : " << z_mean );
    DEBUG_MSG( "Scale factor : " << scale_factor << " inv: " << 1.0f / scale_factor );
    DEBUG_MSG( "diameter: " << containing_diameter );
}

bool PartialViewComputer::presets() {

    if(g_vertex_buffer_data.empty()) {
        std::cerr << "No mesh loaded." << std::endl;
        return false;
    }

    // Initialize GLEW
    if (glewInit() != GLEW_OK) {
        fprintf(stderr, "Failed to initialize GLEW\n");
        return false;
    }

    // Dark blue background
    glClearColor(1.0f, 1.0f, 1.0f, 0.0f);

    // Enable depth test
    glEnable(GL_DEPTH_TEST);
    // Accept fragment if it closer to the camera than the former one
    glDepthFunc(GL_LESS);

    // Create and compile our GLSL program from the shaders
    const char* vertex_sharder = "../shaders/TransformVertexShader.vertexshader";
    const char* fragment_sharder = "../shaders/ColorFragmentShader.fragmentshader";

    programID = LoadShaders( vertex_sharder, fragment_sharder );

    // Get a handle for our "MVP" uniform
    MVP_ID = glGetUniformLocation(programID, "MVP");
    SCALE_ID = glGetUniformLocation(programID, "SCALE");
    MV_ID = glGetUniformLocation(programID, "MV");

    // Get a handle for our buffers
    vertexPosition_modelspaceID = glGetAttribLocation(programID, "vertexPosition_modelspace");

    glGenBuffers(1, &vertexbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
    glBufferData(GL_ARRAY_BUFFER, g_vertex_buffer_data.size() * sizeof(glm::vec3), &g_vertex_buffer_data[0], GL_STATIC_DRAW);

    return true;
}

glm::mat4 PartialViewComputer::ViewMatrix(float theta, float phi) {
    //coodinate on a 1 radius sphere
    glm::vec3 cam_pos(std::sin(theta) * std::cos(phi), std::sin(theta) * std::sin(phi), std::cos(theta));

    DEBUG_MSG( "Cam Info :" << cam_pos.x << " " << cam_pos.y << " " << cam_pos.z );
    return glm::lookAt(
                cam_pos, // Coordinates of the camera, in World Space
                glm::vec3(0,0,0), // and looks at there in world space
                glm::vec3(0,1,0) // Head is up (set to 0,-1,0 to look upside-down)
                );
}

glm::mat4 PartialViewComputer::ProjectionMatrix(float theta, float phi) {
    double aspect = width / height;
    GLdouble horizontal = containing_diameter;
    GLdouble vertical = containing_diameter;
    if ( aspect < 1.0 ) { // window taller than wide
        vertical /= aspect;
    } else {
        horizontal *= aspect;
    }
    return glm::ortho<float>(- horizontal, horizontal, - vertical, vertical, - containing_diameter, containing_diameter);
}

void PartialViewComputer::init_MVP(float theta, float phi) {

    glm::mat4 Model   = glm::translate(- centroid);
    glm::mat4 View    = ViewMatrix(theta, phi);
    glm::mat4 Projection = ProjectionMatrix(theta, phi);
    // Our ModelViewProjection : multiplication of our 3 matrices
    MVP = Projection * View * Model;

    // This matrix will scale the model to make vertice corrdinates belowg to [0,1]
    glm::vec3 translation(min_relative_position[0],
            min_relative_position[1],
            min_relative_position[2]);
    SCALING = glm::translate(-translation) * glm::scale(scale_factor, scale_factor, scale_factor);

}

DefaultPointCloud PartialViewComputer::compute_view(float theta, float phi){
    if (!(glfwContextSet || windowsLessContextSet)) {
        setGLFWContext();
        //setWindowlessContext(); //buggy
    }
    init_MVP(theta,phi);
    draw();
    DefaultPointCloud cloud;
    build_cloud_from_framebuffer(&cloud);
    //build_cloud_from_pixelbuffer(&cloud);

    DEBUG_MSG( width * height << " pixels." );
    DEBUG_MSG( cloud.size() << " points in cloud." );
    assert(!cloud.empty());
    return cloud;
}

void PartialViewComputer::draw() {
    // Clear the screen
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Use our shader
    glUseProgram(programID);

    glUniformMatrix4fv(MVP_ID, 1, GL_FALSE, &MVP[0][0]);
    glUniformMatrix4fv(MV_ID, 1, GL_FALSE, &SCALING[0][0]);
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
    glDrawArrays(GL_TRIANGLES, 0, g_vertex_buffer_data.size()*3);

    glDisableVertexAttribArray(vertexPosition_modelspaceID);
}

void PartialViewComputer::displayMesh(float theta, float phi) {
    setGLFWContext();
    init_MVP(theta,phi);
    do{
        draw();
        // Swap buffers
        glfwSwapBuffers();

    } // Check if the ESC key was pressed or the window was closed
    while( glfwGetKey( GLFW_KEY_ESC ) != GLFW_PRESS &&
           glfwGetWindowParam( GLFW_OPENED ) );
}

void PartialViewComputer::pixel_vector_to_pointcloud(std::vector<float> * data, DefaultPointCloud * cloud) {
    assert( data->size() % 3 == 0); //data should be successive x,y,z values
    cloud->clear(); //empty the cloud

    for(std::vector<float>::iterator it = data->begin(); it < data->end(); ++it) {
        float x = *it;
        ++it;
        float y = *it;
        ++it;
        float z = *it;

        bool isBackground = (x == 1.0f && y == 1.0f && z == 1.0f);
        if(!isBackground)
            //this is to fix, due to some border effect, some pix do not represent the position
            //because the background is white they are augmented. Bug only with GLFW ??*
            /*if ( min_relative_position[0] <= x && x <= max_relative_position[0]
                && min_relative_position[1] <= y && y <= max_relative_position[1]
                && min_relative_position[2] <= z && z <= max_relative_position[2]
                )*/
            cloud->push_back(pcl::PointXYZ(x,y,z));
    }
}

void PartialViewComputer::build_cloud_from_framebuffer(DefaultPointCloud * cloud) {

    GLuint fbo;
    glGenFramebuffers(1,&fbo);

    glFramebufferRenderbuffer(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, vertexbuffer);

    //Before drawing
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER,fbo);

    draw();

    //after drawing
    std::vector<float> data(width*height*3);
    glReadBuffer(GL_COLOR_ATTACHMENT0);
    glReadPixels(0,0,width,height,GL_RGB,GL_FLOAT,&data[0]);

    pixel_vector_to_pointcloud( &data, cloud);

    // Return to onscreen rendering:
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER,0);
    //At deinit:
    glDeleteFramebuffers(1,&fbo);
}

/*
void PartialViewComputer::build_cloud_from_pixelbuffer(DefaultPointCloud * cloud) {
    std::vector<float> data(width*height*3);
    glReadBuffer(GL_BACK);
    glReadPixels(0,0,width,height,GL_RGB,GL_FLOAT,&data[0]);
    pixel_vector_to_pointcloud( &data, cloud);
}*/

bool PartialViewComputer::setGLFWContext(const char* window_name) {

    if(glfwContextSet)
        return true;
    if(windowsLessContextSet)
        free_gpu();

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
        return false;
    }

    glfwSetWindowTitle( window_name );
    // Ensure we can capture the escape key being pressed below
    glfwEnable( GLFW_STICKY_KEYS );

    DEBUG_MSG( "GLFW context set." );
    glfwContextSet = true;
    return glfwContextSet && presets();
}

bool PartialViewComputer::setWindowlessContext() {

    // the results is different than the one expected. Seems to be some missing points in the resulting cloud.
    std::cerr << "The function setWindowlessContext is buggy please don't use it." << std::endl;

    if(windowsLessContextSet)
        return true;
    if(glfwContextSet)
        free_gpu();

    typedef GLXContext (*glXCreateContextAttribsARBProc)(Display*, GLXFBConfig, GLXContext, Bool, const int*);
    typedef Bool (*glXMakeContextCurrentARBProc)(Display*, GLXDrawable, GLXDrawable, GLXContext);
    static glXCreateContextAttribsARBProc glXCreateContextAttribsARB = NULL;
    static glXMakeContextCurrentARBProc   glXMakeContextCurrentARB   = NULL;

    glXCreateContextAttribsARB = (glXCreateContextAttribsARBProc) glXGetProcAddressARB( (const GLubyte *) "glXCreateContextAttribsARB" );
    glXMakeContextCurrentARB   = (glXMakeContextCurrentARBProc)   glXGetProcAddressARB( (const GLubyte *) "glXMakeContextCurrent"      );
    assert( glXCreateContextAttribsARB != NULL);
    assert( glXMakeContextCurrentARB != NULL);

    const char *displayName = NULL;
    Display* display = XOpenDisplay( displayName );
    assert( display != NULL);

    static int visualAttribs[] = { None };
    int numberOfFramebufferConfigurations = 0;
    GLXFBConfig* fbConfigs = glXChooseFBConfig( display, DefaultScreen(display), visualAttribs, &numberOfFramebufferConfigurations );
    assert( fbConfigs != NULL );

    int context_attribs[] = {
        GLX_CONTEXT_MAJOR_VERSION_ARB, 3,
        GLX_CONTEXT_MINOR_VERSION_ARB, 0,
        GLX_CONTEXT_FLAGS_ARB, GLX_CONTEXT_DEBUG_BIT_ARB,
        GLX_CONTEXT_PROFILE_MASK_ARB, GLX_CONTEXT_CORE_PROFILE_BIT_ARB,
        None
    };

    GLXContext openGLContext = glXCreateContextAttribsARB( display, fbConfigs[0], 0, True, context_attribs);


    int pbufferAttribs[] = {
        GLX_PBUFFER_WIDTH,  width,
        GLX_PBUFFER_HEIGHT, height,
        None
    };
    GLXPbuffer pbuffer = glXCreatePbuffer( display, fbConfigs[0], pbufferAttribs );

    // clean up:
    XFree( fbConfigs );
    XSync( display, False );

    windowsLessContextSet = glXMakeContextCurrent( display, pbuffer, pbuffer, openGLContext );
    return windowsLessContextSet && presets();
}

void PartialViewComputer::free_gpu() {
    // Cleanup VBO and shader
    glDeleteBuffers(1, &vertexbuffer);
    glDeleteProgram(programID);
    DEBUG_MSG( "GPU Freed." );
    // Close OpenGL window and terminate GLFW
    if(glfwContextSet)
        glfwTerminate();
    windowsLessContextSet = glfwContextSet = false;
}
