from django.test import TestCase

from sketchup_models.models import SketchupModel
from partial_view.models import PartialView

from unittest import skip

class TestPartialView(TestCase):

    def setUp(self):
        test_model = SketchupModel()
        test_model.google_id = "test1"
        test_model.tags = ["tag1", "tag2"]
        test_model.title = "title1"
        test_model.text = "Description of 'title1' SketchupModel."
        test_model.mesh = file("sketchup_models/fixtures/mesh_can.tri").read()
        test_model.save()
        self.test_model = SketchupModel.find_google_id("test1")

    def test_write_and_read(self):
        """
        Tests writing then reading of a PointCloudStorage and ShapeDistribution.
        """
        # Generate the pointcloud
        view  = PartialView(model=self.test_model, theta=0.0, phi=0.0)
        self.assertTrue( view.pointcloud.size() > 0 )

        view.save()
        self.assertEqual( PartialView.objects.count(), 1 )

        restored_view = PartialView.objects.get(pk=view.pk)
        self.assertEqual( restored_view.theta, view.theta )
        self.assertEqual( restored_view.phi, view.phi )
        self.assertEqual( restored_view.model, view.model )
        self.assertEqual( restored_view.pointcloud.size(), view.pointcloud.size() )

    def test_model_validation(self):
        """
        Tests the validation rules for the view.
        """
        self.assertEqual( PartialView.objects.count(), 0 )
        PartialView(model=self.test_model, theta=0.0, phi=1.345).save()
        self.assertEqual( PartialView.objects.count(), 1 )

        # If the set model, theta, phi is the same the view should not save
        PartialView(model=self.test_model, theta=0.0, phi=1.345).save()
        self.assertEqual( PartialView.objects.count(), 1 )
        PartialView(model=self.test_model, theta=1.0, phi=1.345).save()
        self.assertEqual( PartialView.objects.count(), 2 )

    @skip("long test activate only when needed")
    def test_empty_pointcloud(self):
        """
        The pointcloud is sometimes returned empty.
        It seems to be random and about 1/10 of the time
        We test here the stability.
        """
        test_model = SketchupModel.find_google_id("fb01ac920e6b68fd7236c5b88164c0b8")
        number_of_attempt = 10
        for i in range(number_of_attempt):
            print "Compute cloud {} on {}".format(i, number_of_attempt)
            # Generate the pointcloud
            PartialView.compute_all_views( test_model )
