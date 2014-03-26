"""
This file demonstrates writing tests using the unittest module. These will pass
when you run "manage.py test".

Replace this with more appropriate tests for your application.
"""

from django.test import TestCase
from pointcloud.models import PointCloud
import tempfile, os

class SimpleTest(TestCase):

    def test_basis(self):
        cloud = PointCloud()
        self.assertEqual(cloud.size(), 0)

    # from unittest import skip
    # @skip("load_pcd does not work.")
    def test_save_load_pcd(self):
        pcd_file = open("pointcloud/fixtures/cloud.pcd")
        pointcloud = PointCloud.load_pcd(pcd_file.name)
        self.assertEqual( pointcloud.size(), 8 )
        with tempfile.NamedTemporaryFile() as f:
            pointcloud.save_pcd(f.name)
            f.seek(0)
            self.assertTrue( f.read() == pcd_file.read() )

    def test_read_and_write(self):
        pass
        # pointcloud = PointCloud.load_pcd( "pointcloud/fixtures/can.pcd")

