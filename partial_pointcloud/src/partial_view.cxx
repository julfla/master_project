#include <iostream>
#include <string>

#include <boost/program_options.hpp> //used to parse command line arguments
#include "partial_pointcloud/partial_view.h"
#include "descriptors/shape_distribution.h"

namespace po = boost::program_options;
using namespace std;

#define SQRT_NUMBER_VIEWS 8

bool process_cloud(string output_path, string output_format,
                   pcl::PointCloud<pcl::PointXYZ> * cloud) {

    if(output_format == "pcd") {
        if (cloud->empty()) {
            cerr << "Cloud is empty. It will not be saved." << endl;
            return false;
        }
        pcl::io::savePCDFileASCII (output_path, *cloud);
    } else if( output_format == "dist" ) {
        Distribution distribution(cloud);
        if(output_path != "") {
            std::ofstream ofs(output_path.c_str());
            boost::archive::text_oarchive oa(ofs);
            oa << distribution;
        } else
            boost::archive::text_oarchive(std::cout) << distribution;
    } else if( output_format == "csv") {
        Distribution distribution(cloud);
        if(output_path != "") {
            std::ofstream ofs(output_path.c_str());
            ofs << distribution.to_csv();
        } else
            std::cout << distribution.to_csv();
    } else
        return false;

    return true;
}

int compute_view(PartialViewComputer &comp, float theta, float phi,
                 bool view_image, bool no_clobber, string output_format,
                 string output_path) {
    // check if file already exists
    if ( no_clobber && ifstream(output_path.c_str()) ) {
        return 0;
    } else {
        if (view_image)
            comp.displayMesh(theta, phi);
        cout << "Entropy is " << comp.compute_entropy(theta, phi) << endl;
        if (output_format == "none")  // no output needed
            return 0;
        pcl::PointCloud<pcl::PointXYZ> cloud = comp.compute_view(theta, phi);
        if (!process_cloud(output_path, output_format, &cloud))
            return -1;
    }
    return 0;
}

int main(int argc, char **argv) {

    string input_path, output_path;
    output_path = "";

    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
            ("help,h", "Produce help message")
            ("input,i", po::value<string>(), "Input tri file")
            ("output,o", po::value<string>(), "Ouput pcd file")
            ("output-format,O", po::value<string>()->default_value("pcd"), "Ouput file format pcd|dist|csv")
            ("theta,t", po::value<float>()->default_value( M_PI * 0.30 ), "Theta angle")
            ("phi,p", po::value<float>()->default_value( 2 * M_PI * 0.125 ), "Phi angle")
            ("several,s" , po::bool_switch(), "Compute several partial views (disable phi & theta options)")
            ("view-image,v" , po::bool_switch(), "Display the intermediate partial view")
            ("no-clobber,n", po::bool_switch(), "Do not overwrite an existing file.")
            ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
        cout << desc << endl;
        return 1;
    }

    bool view_image = vm["view-image"].as<bool>();
    bool no_clobber = vm["no-clobber"].as<bool>();
    string output_format = vm["output-format"].as<string>();

    if (vm.count("input")) {
        input_path = vm["input"].as<string>();
    } else {
        cout << "Input file unknown or invalid." << endl;
        cout << desc << endl;
        return 1;
    }

    if (vm.count("output"))
        output_path = vm["output"].as<string>();
    else if (view_image)
        output_format = "none"; // accepted empty parameter if view is displayed
    else {
        cerr << "Output file needed." << endl;
        cerr << desc << endl;
        return 1;
    }


    PartialViewComputer * comp;
    try {
        comp = new PartialViewComputer(input_path);
    }
    catch (exception& e) {
        cerr << e.what() << endl;
        return 1;
    }

    if(vm["several"].as<bool>()) {
        for(int i = 0; i < SQRT_NUMBER_VIEWS; ++i)
            for(int j = 0; j < SQRT_NUMBER_VIEWS; ++j) {
                float theta = M_PI * (float) i / (float) SQRT_NUMBER_VIEWS;
                float phi = 2 * M_PI * (float) j / (float) SQRT_NUMBER_VIEWS;

                string current_output = output_path;
                if( current_output != "" && current_output.find(".") != -1) {
                    stringstream stream;
                    string ext = output_path.substr(output_path.find_last_of("."));
                    string basename = output_path.substr(0, output_path.length() - ext.length());
                    stream << basename << i << j << ext;
                    current_output = stream.str();
                }

                int current_status = compute_view( *comp, theta, phi, view_image, no_clobber, output_format, output_path );
                if (current_status != 0)
                    return current_status;
            }
    } else {
        float theta = vm["theta"].as<float>();
        float phi = vm["phi"].as<float>();
        return compute_view( *comp, theta, phi, view_image, no_clobber, output_format, output_path );
    }

    delete comp;

    return 0;
}



