from django.db import models
from djangotoolbox.fields import EmbeddedModelField
from django_mongodb_engine.fields import GridFSField
from django.utils.six import with_metaclass

from sketchup_models.models import SketchupModel
from shape_distribution.models import ShapeDistribution
from common.libs.libpypartialview import PointCloud, PartialViewComputer
# from common.libs.libpydescriptors import Distribution
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
    view_computer = PartialViewComputer()

    # TODO : should not save if model, theta, or phi blank
    model = models.ForeignKey(SketchupModel)
    theta = models.FloatField()
    phi = models.FloatField()
    pointcloud = PointCloudField()
    distribution = EmbeddedModelField(ShapeDistribution, blank=True, null=True)

    class Meta:
        unique_together = (("model", "theta", "phi"),)

    @staticmethod
    def compute_view(model, theta, phi):
        with tempfile.NamedTemporaryFile() as f :
            f.write( model.mesh )
            f.flush()
            PartialView.view_computer.load_mesh(f.name)

        # Instanciate the model and return
        view = PartialView()
        view.model = model
        view.theta = theta
        view.phi = phi
        view.pointcloud = PartialView.view_computer.compute_view(theta, phi)
        view.distribution = ShapeDistribution.compute( view.pointcloud )
        return view

    @staticmethod
    def compute_all_views(model):
        with tempfile.NamedTemporaryFile() as f:
            f.write( model.mesh )
            f.flush()
            PartialView.view_computer.load_mesh(f.name)
        
        SQRT_NUMBER_VIEWS = 8 # 8 * 8 = 64 views per object
        # from math import pi
        pi = 3.1416
        for i in range(SQRT_NUMBER_VIEWS):
            for j in range(SQRT_NUMBER_VIEWS):
                view = PartialView()
                view.model = model
                view.theta = pi * i / SQRT_NUMBER_VIEWS
                view.phi = 2 * pi * j / SQRT_NUMBER_VIEWS
                view.pointcloud = PartialView.view_computer.compute_view(view.theta, view.phi)
                view.distribution = ShapeDistribution.compute( view.pointcloud )
                view.save()

    @staticmethod
    def display_view(model, theta, phi):
        with tempfile.NamedTemporaryFile() as f :
            f.write( model.mesh )
            f.flush()
            PartialView.view_computer.load_mesh(f.name)
        PartialView.view_computer.display_mesh(theta, phi)
       