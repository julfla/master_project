from django.db import models
from django.utils.six import with_metaclass

from partial_view.models import PartialView

from common.libs.libpydescriptors import ShapeDistribution

class DistributionField(with_metaclass(models.SubfieldBase, models.Field)):
    # Recreate python object from db
    def to_python(self, value):
    	if isinstance(value, ShapeDistribution):
            return value
        else:
            temp = ShapeDistribution()
            if value is not None and len(value) > 0: 
                temp.serialized_data = value.__str__()
            return temp

    # Serialize python object to be stored in db
    def get_prep_value(self, value):
        if isinstance(value, ShapeDistribution):
            return value.serialized_data
        else:
            return value

class ShapeDistribution(models.Model):
    view = models.ForeignKey(PartialView)
    data = DistributionField()
