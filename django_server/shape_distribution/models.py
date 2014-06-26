from django.db import models
from django.utils.six import with_metaclass

from common.libs.libpydescriptors import Distribution

SHAPE_DISTRIBUTION_SIZE = 64


class DistributionField(with_metaclass(models.SubfieldBase, models.Field)):

    """ Django Field to store a ShapeDistribution object. """

    def to_python(self, value):
        """ Recreate python object from db. """
        if isinstance(value, Distribution):
            return value
        if value is None or len(value) == 0:
            return None
        else:
            temp = Distribution()
            temp.serialized_data = value.__str__()
            return temp

    def get_prep_value(self, value):
        """ Serialize python object to be stored in db. """
        if isinstance(value, Distribution):
            return value.serialized_data
        else:
            return value


class ShapeDistribution(models.Model):

    """ Wrapper around the C++ object Distribution. """

    _cpp_object = DistributionField()

    @property
    def serialized_data(self):
        """ Data serialized as a string. """
        return self._cpp_object.serialized_data

    @property
    def as_numpy_array(self):
        """ Data formated in a numpy array for data processing. """
        return self._cpp_object.as_numpy_array

    @staticmethod
    def compute(cloud):
        """ Static method to compute a ShapeDistribution from a PointCloud. """
        temp = ShapeDistribution()
        temp._cpp_object = Distribution(cloud._cpp_pointcloud)
        return temp
