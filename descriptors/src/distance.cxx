#include "shape_distribution.h"
#include <iostream>
#include <string>

using namespace std;

int usage() {
    cout << "Wrong number of arguments." << endl;
    cout << "Usage is <file .dist 1> <file .dist 2>" << endl;
    return 1;
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

        cout << distribution1.distance(distribution2) << std::endl;
        return 0;
    }
}


