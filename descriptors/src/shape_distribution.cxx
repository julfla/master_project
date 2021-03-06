#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <boost/program_options.hpp> //used to parse command line arguments

#include "descriptors/shape_distribution.h"
#include "mesh/mesh.hpp"


namespace po = boost::program_options;
using namespace std;

int main(int argc, char **argv) {

    string input_path, output_path;

    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
            ("help,h", "Produce help message")
            ("input,i", po::value<string>(), "Input pcd file")
            ("output,o", po::value<string>(), "Output file")
            ("output-format,O", po::value<string>()->default_value("archive"), "Ouput file format archive|csv")
            ("no-clobber,n", po::bool_switch(), "Do not overwrite an existing file.")
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

    if (vm.count("output")) {
        output_path = vm["output"].as<string>();
        //check if file exists
        if( ifstream(output_path.c_str()) && vm["no-clobber"].as<bool>() ) {
            return 1;
        }
    }

    // load the pointcloud
    pcl::PointCloud<pcl::PointXYZ> cloud;
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (input_path, cloud) == -1) //* load the file
    {
        cout << "Input file unknown or invalid." << endl;
        cout << desc << endl;
        return -1;
    }
    Distribution distribution(&cloud);

    if (vm["output-format"].as<string>() == "csv") { //export to csv
        if (vm.count("output")) {
            std::ofstream ofs(output_path.c_str());
            ofs << distribution.to_csv();
        }
        else {
            std::cout << distribution.to_csv();
        }
    } else if (vm["output-format"].as<string>() == "archive") {
        if (vm.count("output")) {
            distribution.save_archive(output_path);
        }
        else {
            boost::archive::text_oarchive(std::cout) << distribution;
        }
    } else {
        cout << desc << endl;
        return 1;
    }

    return 0;
}
