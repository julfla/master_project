#ifndef SHADER_HELPER_HPP
#define SHADER_HELPER_HPP

#include <string>
#include <vector>
#include <iostream>
#include <fstream>

#include <GL/glew.h>

static char const * parseShaderSource(const char * path) {
    std::ifstream ifs(path, std::ios::in);
    if(!ifs.good()) {
        throw std::ifstream::failure("Impossible to open" + std::string(path) + ".");
    }
    std::string shaderCode = "";
    std::string line = "";
    while(getline(ifs, line))
        if(!line.length() == 0)
            shaderCode += line + "\n";
    ifs.close();
    shaderCode +=  "\n";
    return shaderCode.c_str();
}

static void compileShader(const char * source_path, GLuint SharderID) {

    std::cout << "Compiling shader : " << source_path << std::endl;
    char const * sourceCode = parseShaderSource(source_path);
    glShaderSource(SharderID, 1, &sourceCode , NULL);
    glCompileShader(SharderID); //I don't know why but this line seems to output empty lines to the terminal...
    // Check the program
    GLint Result = GL_FALSE;
    int InfoLogLength = 0;
    glGetProgramiv(SharderID, GL_LINK_STATUS, &Result);
    glGetProgramiv(SharderID, GL_INFO_LOG_LENGTH, &InfoLogLength);
    if ( InfoLogLength > 0 ){
        std::vector<char> ProgramErrorMessage(InfoLogLength+1);
        glGetProgramInfoLog(SharderID, InfoLogLength, NULL, &ProgramErrorMessage[0]);
        printf("%s", &ProgramErrorMessage[0]);
    }
}

static GLuint LoadShaders(const char * vertex_file_path,const char * fragment_file_path){

    //Compiling Vertex Shader
    GLuint VertexShaderID = glCreateShader(GL_VERTEX_SHADER);
    compileShader(vertex_file_path, VertexShaderID);

    // Compiling Fragment Shader
    GLuint FragmentShaderID = glCreateShader(GL_FRAGMENT_SHADER);
    compileShader(fragment_file_path, FragmentShaderID);

    // Link the program
    GLuint ProgramID = glCreateProgram();
    glAttachShader(ProgramID, VertexShaderID);
    glAttachShader(ProgramID, FragmentShaderID);
    glLinkProgram(ProgramID);

    glDeleteShader(VertexShaderID);
    glDeleteShader(FragmentShaderID);

    return ProgramID;
}

static GLuint LoadShaders(const std::string vertex_file_path,const std::string fragment_file_path) {
    return LoadShaders(vertex_file_path.c_str(), fragment_file_path.c_str());
}

#endif //SHADER_HELPER_HPP