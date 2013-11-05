#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

void usage(int argc, char** argv) {
    std::cout << "Input path is missing or incorrect." << std::endl;
    std::cout << "Usage is : " << argv[0] << " <input pcd file> ." << std::endl;
}

int main (int argc, char** argv)
{
    if(argc != 2) {
        usage(argc, argv);
        return -1;
    }

    char* path = argv[1];

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    int load_result = pcl::io::loadPCDFile (path, *cloud);

    if( load_result != 0) {
        usage(argc, argv);
        return load_result;
    }

    pcl::visualization::CloudViewer viewer("Cloud Viewer");

    viewer.showCloud(cloud);

    while (!viewer.wasStopped ())
        boost::this_thread::sleep (boost::posix_time::millisec(20));
    return 0;
}
