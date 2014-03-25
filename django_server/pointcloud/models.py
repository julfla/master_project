from django.db import models

from django.utils.six import with_metaclass

from common.libs.libpypartialview import PointCloud as CppPointCloud

class PointCloudField(with_metaclass(models.SubfieldBase, models.Field)):
    # Recreate python object from db
    def to_python(self, value):
        if isinstance(value, CppPointCloud):
            return value
        else:
            temp = CppPointCloud()
            if value is not None and len(value) > 0: 
                temp.serialized_data = value.__str__()
            return temp

    # Serialize python object to be stored in db
    def get_prep_value(self, value):
        if isinstance(value, CppPointCloud):
            return value.serialized_data
        else:
            return value

class PointCloud(models.Model):
    _cpp_pointcloud = PointCloudField()

    def size(self):
    	return self._cpp_pointcloud.size()

    @property
    def serialized_data(self):
    	return self._cpp_pointcloud.serialized_data

    def save_pcd(self, path):
        return self._cpp_pointcloud.save_pcd(path)