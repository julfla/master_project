from django.db import models
from djangotoolbox.fields import EmbeddedModelField
from django_mongodb_engine.fields import GridFSField
from django.utils.six import with_metaclass

from sketchup_models.models import SketchupModel
from shape_distribution.models import ShapeDistribution
from pointcloud.models import PointCloud
from common.libs.libpypartialview import PartialViewComputer
# from common.libs.libpydescriptors import Distribution
import tempfile

class PartialView(models.Model):
    view_computer = PartialViewComputer()

    # TODO : should not save if model, theta, or phi blank
    model = models.ForeignKey(SketchupModel)
    theta = models.FloatField()
    phi = models.FloatField()
    pointcloud = EmbeddedModelField(PointCloud, blank=True, null=True)
    distribution = EmbeddedModelField(ShapeDistribution, blank=True, null=True)

    class Meta:
        unique_together = (("model", "theta", "phi"),)

    @staticmethod
    def compute_view(model, theta, phi):
        print "Computing views for model {}...".format( model )
        with tempfile.NamedTemporaryFile() as f :
            f.write( model.mesh )
            f.flush()
            PartialView.view_computer.load_mesh(f.name)

        # Instanciate the model and return
        view = PartialView()
        view.model = model
        view.theta = theta
        view.phi = phi
        view.pointcloud = PointCloud()
        cpp_cloud = PartialView.view_computer.compute_view(theta, phi)
        view.pointcloud._cpp_pointcloud = cpp_cloud
        view.distribution = ShapeDistribution.compute( view.pointcloud )
        return view

    @staticmethod
    def compute_all_views(model):
        with tempfile.NamedTemporaryFile() as f:
            f.write( model.mesh )
            f.flush()
            print "loading mesh from file({})".format(f.name)
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
       