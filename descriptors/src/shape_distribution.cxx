#include "shape_distribution.h"
#include "mesh.h"
#include <iostream>
#include <string>

#include <boost/program_options.hpp> //used to parse command line arguments

namespace po = boost::program_options;
using namespace std;

int main(int argc, char **argv) {

    string input_path;
    int number_points = 0;

    // Declare the supported options.
    po::
            options_description desc("Allowed options");
    desc.add_options()
            ("help,h", "Produce help message")
            ("input,i", po::value<string>(), "Input file")
            ("output,o", po::value<string>(), "Output file")
            ("csv,c", "Export to a csv intead of a lib boost archive, use it with conv for convertion")
            ("convert", "Convert the file instead of computing it")
            ;


    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
        cout << desc << endl;
        return 1;
    }

    if (vm.count("input")) {
        input_path = vm["input"].as<string>();
    } else {
        cout << "Input file unknown or invalid." << endl;
        cout << desc << endl;
        return 1;
    }

    Distribution distribution;
    if(vm.count("convert")) {
        // load data from archive
        std::ifstream ifs(input_path.c_str());
        boost::archive::text_iarchive oa(ifs);
        oa >> distribution;
    } else
        distribution = Distribution(Mesh(input_path));

    if(vm.count("csv")) { //export to csv
        if(vm.count("output")) {
            std::ofstream ofs(vm["output"].as<string>().c_str());
            ofs << distribution.to_csv();
        }
        else
            std::cout << distribution.to_csv();
    } else {
        if(vm.count("output")) {
            std::ofstream ofs(vm["output"].as<string>().c_str());
            boost::archive::text_oarchive oa(ofs);
            oa << distribution;
        }
        else
            boost::archive::text_oarchive(std::cout) << distribution;
    }

    return 0;
}
