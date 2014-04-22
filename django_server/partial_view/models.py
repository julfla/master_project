from django.db import models
from djangotoolbox.fields import EmbeddedModelField
from django_mongodb_engine.fields import GridFSField
from django.utils.six import with_metaclass

from sketchup_models.models import SketchupModel
from shape_distribution.models import ShapeDistribution
from pointcloud.models import PointCloud


class PartialCloudComputer():
    from common.libs.libpypartialview import PartialViewComputer as CppPartialViewComputer
    _cpp_computer = CppPartialViewComputer()
    _loaded_model_id = None

    def load_model(self, sketchup_model):
        print "Loading model {} into PartialCloudComputer...".format( sketchup_model )
        import tempfile
        with tempfile.NamedTemporaryFile() as f :
            f.write( sketchup_model.mesh )
            f.flush()
            self._cpp_computer.load_mesh(f.name)
        PartialCloudComputer._loaded_model_id = sketchup_model.google_id

    def compute_pointcloud(self, sketchup_model, theta, phi):
        if self._loaded_model_id != sketchup_model.google_id :
            self.load_model(sketchup_model)
        print "Computing view for model {} : t={}, p={}".format(
            sketchup_model.google_id, theta, phi)
        cloud = PointCloud()
        cloud._cpp_pointcloud = self._cpp_computer.compute_view(theta, phi)
        return cloud

    def display_view(self, sketchup_model, theta, phi):
        if self._loaded_model_id != sketchup_model.google_id :
            self.load_model(sketchup_model)
        self._cpp_computer.display_mesh(theta, phi)

class PartialView(models.Model):

    # TODO : should not save if model, theta, or phi blank
    model = models.ForeignKey(SketchupModel)
    theta = models.FloatField()
    phi = models.FloatField()
    _distribution = EmbeddedModelField(ShapeDistribution, blank=True, null=True)

    class Meta:
        unique_together = (("model", "theta", "phi"),)

    @property
    def distribution(self):
        if self._distribution == None:
            self._distribution = ShapeDistribution.compute( self.pointcloud )
        return self._distribution

    @property
    def pointcloud(self):
        if not hasattr(self, '_pointcloud'):
            self._pointcloud = PartialCloudComputer().compute_pointcloud(self.model, self.theta, self.phi)
        return self._pointcloud

    @staticmethod
    def compute_all_views(model):        
        SQRT_NUMBER_VIEWS = 8 # 8 * 8 = 64 views per object
        pi = 3.1416
        for i in range(SQRT_NUMBER_VIEWS):
            for j in range(SQRT_NUMBER_VIEWS):
                print "Computing view {}_{} for model {}".format(i,j,model.google_id)
                theta = pi * i / SQRT_NUMBER_VIEWS
                phi = 2 * pi * j / SQRT_NUMBER_VIEWS
                view = PartialView(model=model, theta=theta, phi=phi)
                view.distribution # force computation of distribution while mesh is loaded
                view.save()

    @staticmethod
    def display_view(model, theta, phi):
        PartialCloudComputer().display_view(model, theta, phi)
    