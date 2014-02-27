from django.db import models
from django.utils.six import with_metaclass

from common.libs.libpydescriptors import ShapeDistribution

class DistributionField(with_metaclass(models.SubfieldBase, models.Field)):
    # Recreate python object from db
    def to_python(self, value):
    	if isinstance(value, ShapeDistribution):
            return value
        if value is None or len( value ) == 0:
        	return None
        else:
            temp = ShapeDistribution()
            temp.serialized_data = value.__str__()
            return temp

    # Serialize python object to be stored in db
    def get_prep_value(self, value):
        if isinstance(value, ShapeDistribution):
            return value.serialized_data
        else:
            return value

class ShapeDistribution(models.Model):
    data = DistributionField()
