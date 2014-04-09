#ifndef DEBUG_HELPER_HPP
#define DEBUG_HELPER_HPP


// The macro DEBUG_MSG will output a message only when the build os debug.
// param : ostream
#ifndef NDEBUG
#include <iostream>
#define DEBUG_MSG(str) do { std::cout << str << std::endl; } while( false )
#else
#define DEBUG_MSG(str) do { } while ( false )
#endif


#endif // DEBUG_HELPER_HPP