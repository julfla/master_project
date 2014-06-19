""" Tests definitions of pointcloud app. """

from django.test import TestCase
from pointcloud.models import PointCloud
import tempfile


class SimpleTest(TestCase):

    """ Test case for the application. """

    def test_basis(self):
        """ Test init of a PointCloud. """
        cloud = PointCloud()
        self.assertEqual(cloud.size(), 0)

    def test_save_load_pcd(self):
        """ Test that PointClouds can be save and load functions. """
        pcd_file = open("pointcloud/fixtures/cloud.pcd")
        pointcloud = PointCloud.load_pcd(pcd_file.name)
        self.assertEqual(pointcloud.size(), 8)
        with tempfile.NamedTemporaryFile() as temp_file:
            pointcloud.save_pcd(temp_file.name)
            temp_file.seek(0)
            self.assertTrue(temp_file.read() == pcd_file.read())
