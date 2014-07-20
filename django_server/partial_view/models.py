from django.db import models
from djangotoolbox.fields import EmbeddedModelField
from django_mongodb_engine.fields import GridFSField
from django.utils.six import with_metaclass

from sketchup_models.models import SketchupModel
from shape_distribution.models import ShapeDistribution
from pointcloud.models import PointCloud


def lazy_property(fn):
    """ Lazy property decorator. """
    attr_name = '_lazy_' + fn.__name__

    @property
    def _lazy_property(self):
        if not hasattr(self, attr_name):
            setattr(self, attr_name, fn(self))
        return getattr(self, attr_name)
    return _lazy_property


class PartialCloudComputer(object):

    def __init__(self):
        from common.libs import libpypartialview
        self._cpp_computer = libpypartialview.PartialViewComputer()
        self._loaded_model_id = None

    def load_model(self, sketchup_model):
        if self._loaded_model_id == sketchup_model.google_id:
            return
        print "Loading model {} into PartialCloudComputer...".format(
            sketchup_model)
        import tempfile
        with tempfile.NamedTemporaryFile() as temp_file:
            temp_file.write(sketchup_model.mesh)
            temp_file.flush()
            self._cpp_computer.load_mesh(temp_file.name)
        self._loaded_model_id = sketchup_model.google_id

    def compute_pointcloud(self, sketchup_model, theta, phi):
        self.load_model(sketchup_model)
        print "Computing view for model {} : t={}, p={}".format(
            sketchup_model.google_id, theta, phi)
        cloud = PointCloud()
        cloud._cpp_pointcloud = self._cpp_computer.compute_view(theta, phi)
        return cloud

    def compute_entropy(self, sketchup_model, theta, phi):
        self.load_model(sketchup_model)
        return self._cpp_computer.compute_entropy(theta, phi)

    def display_view(self, sketchup_model, theta, phi):
        self.load_model(sketchup_model)
        self._cpp_computer.display_mesh(theta, phi)


class PartialView(models.Model):

    model = models.ForeignKey(SketchupModel)
    theta = models.FloatField()
    phi = models.FloatField()
    entropy = models.FloatField(null=True)
    _distribution = EmbeddedModelField(ShapeDistribution, null=True)

    class Meta:
        unique_together = (("model", "theta", "phi"),)

    @lazy_property
    def partial_cloud_computer(self):
        return PartialCloudComputer()

    @partial_cloud_computer.setter
    def partial_cloud_computer(self, value):
        setattr(self, '_lazy_partial_cloud_computer', value)

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
            self._pointcloud = self.partial_cloud_computer.compute_pointcloud(
                self.model, self.theta, self.phi)
            self.entropy = self.partial_cloud_computer.compute_entropy(
                self.model, self.theta, self.phi)
        return self._pointcloud

    @staticmethod
    def compute_all_views(model):
        SQRT_NUMBER_VIEWS = 8  # 8 * 8 = 64 views per object
        pi = 3.1416
        partial_cloud_computer = PartialCloudComputer()
        for i in range(SQRT_NUMBER_VIEWS):
            for j in range(SQRT_NUMBER_VIEWS):
                print "Computing view {}_{} for model {}".format(
                    i, j, model.google_id)
                theta = pi * i / SQRT_NUMBER_VIEWS
                phi = 2 * pi * j / SQRT_NUMBER_VIEWS
                view = PartialView(model=model, theta=theta, phi=phi)
                setattr(view, 'partial_cloud_computer',
                        partial_cloud_computer)
                # force computation of distribution while mesh is loaded
                view.distribution
                view.save()

    def display(self):
        self.partial_cloud_computer.display_view(
            self.model, self.theta, self.phi)
