#ifndef SERIALIZATION_POINTCLOUD_HPP
#define SERIALIZATION_POINTCLOUD_HPP

#include <boost/serialization/split_free.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>

namespace boost{
  namespace serialization {

template<typename PointT, class Archive>
    inline void serialize(Archive & ar, pcl::PointCloud<PointT> & cloud, const unsigned int file_version)
    {split_free(ar, cloud, file_version);}

template<typename PointT, class Archive>
    void save(Archive & ar, const pcl::PointCloud<PointT> & cloud, unsigned int version)
    {
      int size = cloud.size();
      ar << size;
      for (int i = 0; i < size; ++i)
      {
        ar << cloud.points[i];
      }
    }

template<typename PointT, class Archive>
    void load(Archive & ar, pcl::PointCloud<PointT> & cloud, unsigned int version)
    {
      cloud.clear();
      int size;
      ar >> size;
      for (int i = 0; i < size; ++i)
      {
        PointT point;
        ar >> point;
        cloud.push_back(point);
      }
    }

template<class Archive>
    void serialize(Archive & ar, pcl::PointXYZ & point, const unsigned int version)
    {
      ar & point.x;
      ar & point.y;
      ar & point.z;
    }

template<class Archive>
    void serialize(Archive & ar, pcl::PointXYZRGBA & point, const unsigned int version)
    {
      ar & point.x;
      ar & point.y;
      ar & point.z;
      ar & point.r;
      ar & point.g;
      ar & point.b;
    }


}//namespace serialization
}//namespace boost
#endif // SERIALIZATION_POINTCLOUD_HPP