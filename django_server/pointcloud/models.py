""" Definition of the PointCloud model. """

from django.db import models
from django.utils.six import with_metaclass
from common.libs.libpypartialview import PointCloud as CppPointCloud


class PointCloudField(with_metaclass(models.SubfieldBase, models.Field)):

    """ Django field to contain a PointCloud object. """

    def to_python(self, value):
        """ Recreate python object from db. """
        if isinstance(value, CppPointCloud):
            return value
        else:
            temp = CppPointCloud()
            if value is not None and len(value) > 0:
                temp.serialized_data = value.__str__()
            return temp

    def get_prep_value(self, value):
        """ Serialize python object to be stored in db. """
        if isinstance(value, CppPointCloud):
            return value.serialized_data
        else:
            return value


class PointCloud(models.Model):

    """ This class is a wrapper of the cpp pointcloud objet. """

    _cpp_pointcloud = PointCloudField()

    def size(self):
        """ Return the number of points in the cloud. """
        return self._cpp_pointcloud.size()

    @property
    def serialized_data(self):
        """ Serialize the data to a string object. """
        return self._cpp_pointcloud.serialized_data

    def save_pcd(self, path):
        """ Save the point cloud into the pcd_file <path>. """
        return self._cpp_pointcloud.save_pcd(path)

    @staticmethod
    def load_pcd(path):
        """ Load a point cloud from the pcd_file <path>. """
        temp = PointCloud()
        temp._cpp_pointcloud = CppPointCloud.load_pcd(path)
        return temp
