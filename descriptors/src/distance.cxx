#include "descriptors/shape_distribution.h"
#include <iostream>

#include <cstdlib>

using namespace std;

int usage() {
    cout << "Wrong number of arguments." << endl;
    cout << "Usage is <file .dist 1> <file .dist 2>" << endl;
    return 1;
}

double distance_abs(double n1, double n2) {return std::abs(n1 - n2);}
double distance_rel(double n1, double n2) {(n1 - n2) * (n1 - n2) / (n1 + n2);}
double distance_log(double n1, double n2) {
    if(n1 < 1e-6)
        return n2 * std::log(0.5);
    if( n2 < 1e-6 )
        return n1 * std::log(0.5);
    return 0.5*(n1+n2) * std::log(0.5*(n1+n2) / std::sqrt(n1 * n2) );
}

int main(int argc, char **argv) {

    if(argc < 3) {
        return usage();
    }
    else {

        char* file1 = argv[1];
        char* file2 = argv[2];

        Distribution distribution1 = Distribution::load_archive(file1);
        Distribution distribution2 = Distribution::load_archive(file2);

        cout << distribution1.distance<distance_abs>(distribution2) << std::endl;
        return 0;
    }
}


