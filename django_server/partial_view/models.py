from django.db import models
from djangotoolbox.fields import ListField
from django_mongodb_engine.fields import GridFSField
from django.utils.six import with_metaclass

from sketchup_models.models import SketchupModel
from common.libs.libpypartialview import PointCloud, PartialViewComputer
import tempfile

class PointCloudField(with_metaclass(models.SubfieldBase, models.Field)):
    # Recreate python object from db
    def to_python(self, value):
    	if isinstance(value, PointCloud):
            return value
        else:
            temp = PointCloud()
            if value is not None and len(value) > 0: 
                temp.serialized_data = value.__str__()
            return temp

    # Serialize python object to be stored in db
    def get_prep_value(self, value):
        if isinstance(value, PointCloud):
            return value.serialized_data
        else:
            return value

class PartialView(models.Model):
	# TODO : should not save if model, theta, or phi blank
    model = models.ForeignKey(SketchupModel)
    theta = models.FloatField()
    phi = models.FloatField()
    pointcloud = PointCloudField()

    @staticmethod
    def compute_view(model, theta, phi):
        with tempfile.NamedTemporaryFile() as f :
            f.write( model.mesh.read() )
            f.flush()
            comp = PartialViewComputer()
            comp.load_mesh(f.name)

        # Instanciate the model and return
        view = PartialView()
        view.model = model
        view.theta = theta
        view.phi = phi
        view.pointcloud = comp.compute_view(theta, phi)

    @staticmethod
    def display_view(model, theta, phi):
        with tempfile.NamedTemporaryFile() as f :
            f.write( model.mesh.read() )
            f.flush()
            comp = PartialViewComputer()
            comp.load_mesh(f.name)
        comp.display_mesh(theta, phi)
       