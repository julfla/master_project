""" ShapeDistribution module used a a descriptor in our project. """

from django.db import models
from django.utils.six import with_metaclass
from common.libs import libpydescriptors

SHAPE_DISTRIBUTION_SIZE = libpydescriptors.SHAPE_DISTRIBUTION_SIZE


class DistributionField(with_metaclass(models.SubfieldBase, models.Field)):

    """Custom field to store a ShapeDistribution object. """

    def to_python(self, value):
        """ Recreate python object from db. """
        if isinstance(value, libpydescriptors.Distribution):
            return value
        if value is None or len(value) == 0:
            return None
        else:
            temp = libpydescriptors.Distribution()
            temp.serialized_data = value.__str__()
            return temp

    def get_prep_value(self, value):
        """ Serialize python object to be stored in db. """
        if isinstance(value, libpydescriptors.Distribution):
            return value.serialized_data
        else:
            return value


class ShapeDistribution(models.Model):

    """ Wrapping around the Cpp Distribution Object. """

    _cpp_object = DistributionField()

    @property
    def serialized_data(self):
        """ Wrapping. """
        return self._cpp_object.serialized_data

    @property
    def as_numpy_array(self):
        """ Wrapping. """
        return self._cpp_object.as_numpy_array

    @staticmethod
    def compute(cloud):
        """ Wrapping. """
        temp = ShapeDistribution()
        temp._cpp_object = libpydescriptors.Distribution(cloud._cpp_pointcloud)
        return temp
