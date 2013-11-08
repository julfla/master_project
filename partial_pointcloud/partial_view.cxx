#include <iostream>
#include <string>

#include <boost/program_options.hpp> //used to parse command line arguments
#include "partial_view.h"

namespace po = boost::program_options;
using namespace std;

int main(int argc, char **argv) {

    string input_path, output_path;
    float theta, phi;


    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
            ("help,h", "Produce help message")
            ("input,i", po::value<string>(), "Input tri file")
            ("theta,t", po::value<float>()->default_value(M_PI * 0.30), "Theta angle")
            ("phi,p", po::value<float>()->default_value(2 * M_PI * 0.125), "Phi angle")
            ("output,o", po::value<string>(), "Ouput pcd file")
            ("view-image,v" , po::bool_switch(), "Display the intermediate partial view")
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

    if (vm.count("output"))
        output_path = vm["output"].as<string>();
    else {
            cout << "Output file unknown or invalid." << endl;
            cout << desc << endl;
            return 1;
        }

    theta = vm["theta"].as<float>();
    phi = vm["phi"].as<float>();

    std::cout << "theta: "<< theta << " phi: " << phi << std::endl;

    PartialViewComputer comp(input_path);
    return comp.compute_view(output_path, theta, phi, vm["view-image"].as<bool>());


}
