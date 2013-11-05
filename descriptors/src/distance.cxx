#include "shape_distribution.h"
#include <iostream>
#include <string>

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

        string file1 = argv[1];
        string file2 = argv[2];

        Distribution distribution1, distribution2;

        // load data from archive
        {
            std::ifstream ifs(file1.c_str());
            boost::archive::text_iarchive oa(ifs);
            oa >> distribution1;
        }
        {
            std::ifstream ifs(file2.c_str());
            boost::archive::text_iarchive oa(ifs);
            oa >> distribution2;
        }

        cout << distribution1.distance<distance_abs>(distribution2) << std::endl;
        return 0;
    }
}


