#ifndef SHADER_HPP
#define SHADER_HPP

#ifdef DEBUG
#include <iostream>
#define DEBUG_MSG(str) do { std::cout << str << std::endl; } while( false )
#else
#define DEBUG_MSG(str) do { } while ( false )
#endif

#include <string>
using namespace std;

GLuint LoadShaders(const char * vertex_file_path,const char * fragment_file_path);

#endif
