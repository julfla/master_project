"""
This file demonstrates writing tests using the unittest module. These will pass
when you run "manage.py test".

Replace this with more appropriate tests for your application.
"""

from django.test import TestCase
from sketchup_models.models import SketchupModel


class SimpleTest(TestCase):

    def test_mesh_attribute_can_be_used_as_a_string(self):
        """
        As mesh is store in a gridfsfield, the attribute mesh would
        normally be returned as a GridFSOut object.
        The model has been modified so that it is a string instead.
        """
        model = SketchupModel()
        model.google_id = "test1"
        model.mesh = file("sketchup_models/fixtures/mesh_can.tri").read()

        def error_msg(model):
            return ("The mesh should be of type str but is {} instead.".
                    format(model.mesh.__class__))

        msg = error_msg(model)
        self.assertTrue(isinstance(model.mesh, basestring))
        model.save()
        msg = error_msg(model)
        self.assertTrue(isinstance(model.mesh, basestring))
        model = SketchupModel.find_google_id("test1")
        msg = error_msg(model)
        self.assertTrue(isinstance(model.mesh, basestring), msg=msg)

    def test_google_id_is_unique(self):
        """ Test that the Field google is unique. """
        count = SketchupModel.objects.filter(google_id='test_unique').count()
        self.assertEqual(count, 0)

        SketchupModel(google_id='test_unique').save()
        count = SketchupModel.objects.filter(google_id='test_unique').count()
        self.assertEqual(count, 1)

        SketchupModel(google_id='test_unique').save()
        count = SketchupModel.objects.filter(google_id='test_unique').count()
        self.assertEqual(count, 1)
