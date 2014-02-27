from django.test import TestCase

from sketchup_models.models import SketchupModel
from partial_view.models import PartialView

class TestPartialView(TestCase):
    #fixtures = ["data.json"]

    def test_write_and_read(self):
        """
        Tests writing then reading of a PointCloudStorage.
        """
        test_model = SketchupModel.find_google_id("b88bcf33f25c6cb15b4f129f868dedb")
        view = PartialView()

        # Generate the pointcloud
        view.display_view(test_model, 0.0, 0.0)
        view.compute_view( test_model, 0.0, 0.0)
        self.assertTrue( view.pointcloud.size() > 0 )
        
        view.save()
        self.assertTrue( PartialView.objects.count() == 1 )

        restored_view = PartialView.objects.all()[:1].get()
        self.assertEqual( restored_view.theta, view.theta )
        self.assertEqual( restored_view.phi, view.phi )
        self.assertEqual( restored_view.model, view.model )
        self.assertEqual( restored_view.pointcloud.size(), view.pointcloud.size() )
        # comp = PartialViewComputer()
        #comp.load_mesh("/home/julien/research_project/bin/can1.tri")
        #cloud = comp.compute_view(theta, phi)
        #print cloud.size
        #print cloud.size()

        #print "OKOKOKOK"
        #self.assertEqual(cloud.size, 5)
