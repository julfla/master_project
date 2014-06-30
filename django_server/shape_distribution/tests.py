from django.test import TestCase

from sketchup_models.models import SketchupModel
from partial_view.models import PartialView
from shape_distribution.models import (ShapeDistribution,
                                       SHAPE_DISTRIBUTION_SIZE)
import numpy


class TestShapeDistribution(TestCase):

    """ Test case for the model ShapeDistribution. """

    def setUp(self):
        """ Run before starting testing. """
        # TODO : use fixtures
        test_model = SketchupModel()
        test_model.google_id = "test1"
        test_model.tags = ["tag1", "tag2"]
        test_model.title = "title1"
        test_model.text = "Description of 'title1' SketchupModel."
        test_model.mesh = file("sketchup_models/fixtures/mesh_can.tri").read()
        test_model.save()

        self.test_model = SketchupModel.find_google_id("test1")
        self.view = PartialView(model=self.test_model, theta=0.0, phi=0.0)
        self.distribution = ShapeDistribution.compute(self.view.pointcloud)

    def test_write_and_read(self):
        """ Test writing/reading a ShapeDistribution. """
        self.assertEqual(ShapeDistribution.objects.count(), 0)
        self.distribution.save()
        self.assertEqual(ShapeDistribution.objects.count(), 1)

    def test_data_as_numpy_array(self):
        """ Test that we can use the data as a numpy array. """
        array = self.distribution.as_numpy_array
        self.assertTrue(isinstance(array, numpy.ndarray))
        self.assertEqual(array.size, SHAPE_DISTRIBUTION_SIZE)
