from django.test import TestCase

from sketchup_models.models import SketchupModel
from partial_view.models import PartialView
from shape_distribution.models import ShapeDistribution
from common.libs.libpydescriptors import Distribution
import numpy

from unittest import skip

class TestShapeDistribution(TestCase):
    def setUp(self):

        # TODO : use fixtures, but
        test_model = SketchupModel()
        test_model.google_id = "test1"
        test_model.tags = ["tag1", "tag2"]
        test_model.title = "title1"
        test_model.text = "Description of 'title1' SketchupModel."
        test_model.mesh = file("sketchup_models/fixtures/mesh_can.tri").read()
        test_model.save()

        self.test_model = SketchupModel.find_google_id("test1")
        self.view = PartialView.compute_view(self.test_model, 0.0, 0.0)
        self.distribution = ShapeDistribution.compute( self.view.pointcloud )

    # @skip("Bug Eigen3")
    def test_write_and_read(self):
        """
        Tests writing then reading of a PointCloudStorage and ShapeDistribution.
        """
        self.assertEqual( ShapeDistribution.objects.count(), 0 )
        self.distribution.save()
        self.assertEqual( ShapeDistribution.objects.count(), 1 )

    def test_data_as_numpy_array(self):
        """
        Tests that we can use the data as a numpy array.
        """
        array = self.distribution._cpp_object.as_numpy_array
        self.assertTrue( isinstance(array, numpy.ndarray) )
        self.assertEqual( array.size, 480 )
