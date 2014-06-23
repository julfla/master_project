from django.db import models
from djangotoolbox.fields import EmbeddedModelField
from django_mongodb_engine.fields import GridFSField
from django.utils.six import with_metaclass

from sketchup_models.models import SketchupModel
from shape_distribution.models import ShapeDistribution
from pointcloud.models import PointCloud


class PartialCloudComputer():
    from common.libs.libpypartialview import (PartialViewComputer as
                                              CppPartialViewComputer)
    _cpp_computer = CppPartialViewComputer()
    _loaded_model_id = None

    @staticmethod
    def load_model(sketchup_model):
        if PartialCloudComputer._loaded_model_id == sketchup_model.google_id:
            return
        print "Loading model {} into PartialCloudComputer...".format(
            sketchup_model)
        import tempfile
        with tempfile.NamedTemporaryFile() as temp_file:
            temp_file.write(sketchup_model.mesh)
            temp_file.flush()
            PartialCloudComputer._cpp_computer.load_mesh(temp_file.name)
        PartialCloudComputer._loaded_model_id = sketchup_model.google_id

    @staticmethod
    def compute_pointcloud(sketchup_model, theta, phi):
        PartialCloudComputer.load_model(sketchup_model)
        print "Computing view for model {} : t={}, p={}".format(
            sketchup_model.google_id, theta, phi)
        cloud = PointCloud()
        cloud._cpp_pointcloud = PartialCloudComputer._cpp_computer.compute_view(theta, phi)
        return cloud

    @staticmethod
    def compute_entropy(sketchup_model, theta, phi):
        PartialCloudComputer.load_model(sketchup_model)
        return PartialCloudComputer._cpp_computer.compute_entropy(theta, phi)

    @staticmethod
    def display_view(sketchup_model, theta, phi):
        PartialCloudComputer.load_model(sketchup_model)
        PartialCloudComputer._cpp_computer.display_mesh(theta, phi)


class PartialView(models.Model):

    model = models.ForeignKey(SketchupModel)
    theta = models.FloatField()
    phi = models.FloatField()
    entropy = models.FloatField(null=True)
    _distribution = EmbeddedModelField(ShapeDistribution, null=True)

    class Meta:
        unique_together = (("model", "theta", "phi"),)

    @property
    def distribution(self):
        if not self._distribution:
            self._distribution = ShapeDistribution.compute(self.pointcloud)
        return self._distribution

    @distribution.setter
    def distribution(self, value):
        self._distribution = value

    @property
    def pointcloud(self):
        if not hasattr(self, '_pointcloud'):
            self._pointcloud = PartialCloudComputer.compute_pointcloud(
                self.model, self.theta, self.phi)
            self.entropy = PartialCloudComputer.compute_entropy(
                self.model, self.theta, self.phi)
        return self._pointcloud

    @staticmethod
    def compute_all_views(model):
        SQRT_NUMBER_VIEWS = 8  # 8 * 8 = 64 views per object
        pi = 3.1416
        for i in range(SQRT_NUMBER_VIEWS):
            for j in range(SQRT_NUMBER_VIEWS):
                print "Computing view {}_{} for model {}".format(
                    i, j, model.google_id)
                theta = pi * i / SQRT_NUMBER_VIEWS
                phi = 2 * pi * j / SQRT_NUMBER_VIEWS
                view = PartialView(model=model, theta=theta, phi=phi)
                # force computation of distribution while mesh is loaded
                view.distribution
                view.save()

    def display(self):
        PartialCloudComputer.display_view(self.model, self.theta, self.phi)
