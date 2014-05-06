from django.db import models
from django.utils.six import with_metaclass

from common.libs.libpydescriptors import Distribution

SHAPE_DISTRIBUTION_SIZE = 480

class DistributionField(with_metaclass(models.SubfieldBase, models.Field)):
    # Recreate python object from db
    def to_python(self, value):
    	if isinstance(value, Distribution):
            return value
        if value is None or len( value ) == 0:
        	return None
        else:
            temp = Distribution()
            temp.serialized_data = value.__str__()
            return temp

    # Serialize python object to be stored in db
    def get_prep_value(self, value):
        if isinstance(value, Distribution):
            return value.serialized_data
        else:
            return value

class ShapeDistribution(models.Model):
    _cpp_object = DistributionField()

    @property
    def serialized_data(self):
        return self._cpp_object.serialized_data

    @property
    def as_numpy_array(self):
        return self._cpp_object.as_numpy_array

    @staticmethod
    def compute(cloud):
        temp = ShapeDistribution()
        temp._cpp_object = Distribution( cloud._cpp_pointcloud )
        return temp