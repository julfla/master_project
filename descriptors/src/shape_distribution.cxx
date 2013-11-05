#include "shape_distribution.h"
#include "mesh.h"
#include <iostream>
#include <string>

#include <boost/program_options.hpp> //used to parse command line arguments

namespace po = boost::program_options;
using namespace std;

int main(int argc, char **argv) {

    string input_path, iformat, oformat;

    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
            ("help,h", "Produce help message")
            ("input,i", po::value<string>(), "Input file")
            ("input-format,I", po::value<string>()->default_value("pcd"), "Input file format tri|pcd|archive")
            ("output,o", po::value<string>(), "Output file")
            ("output-format,O", po::value<string>()->default_value("archive"), "Ouput file format archive|csv")
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
    if(vm["input-format"].as<string>() == "archive") {
        // load data from archive
        std::ifstream ifs(input_path.c_str());
        boost::archive::text_iarchive oa(ifs);
        oa >> distribution;
    } else if(vm["input-format"].as<string>() == "tri") {
        distribution = Distribution(Mesh(input_path));
    } else if(vm["input-format"].as<string>() == "pcd") {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        if (pcl::io::loadPCDFile<pcl::PointXYZ> (input_path, cloud) == -1) //* load the file
        {
            cout << "Input file unknown or invalid." << endl;
            cout << desc << endl;
            return -1;
        }
        distribution = Distribution(&cloud);
    }
    else {
        cout << desc << endl;
        return 1;
    }

   if(vm["output-format"].as<string>() == "csv") { //export to csv
        if(vm.count("output")) {
            std::ofstream ofs(vm["output"].as<string>().c_str());
            ofs << distribution.to_csv();
        }
        else
            std::cout << distribution.to_csv();
   } else if(vm["output-format"].as<string>() == "archive") {
        if(vm.count("output")) {
            std::ofstream ofs(vm["output"].as<string>().c_str());
            boost::archive::text_oarchive oa(ofs);
            oa << distribution;
        }
        else
            boost::archive::text_oarchive(std::cout) << distribution;
    } else {
       cout << desc << endl;
       return 1;
   }

    return 0;
}
