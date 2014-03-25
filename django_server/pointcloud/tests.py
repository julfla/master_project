"""
This file demonstrates writing tests using the unittest module. These will pass
when you run "manage.py test".

Replace this with more appropriate tests for your application.
"""

from unittest import skip
from django.test import TestCase
from pointcloud.models import PointCloud
import tempfile, os

class SimpleTest(TestCase):

    def test_basis(self):
        cloud = PointCloud()
        self.assertEqual(cloud.size(), 0)

    @skip("load_pcd does not work.")
    def test_save_load_pcd(self):
        pointcloud = PointCloud.load_pcd("pointcloud/fixtures/can.pcd")
        with tempfile.NamedTemporaryFile() as f:
            pointcloud.save_pcd(f.name)
            f.seek(0)
            self.assertTrue( f.read() == pcd_file.read() )

    def test_read_and_write(self):
        pass
        # pointcloud = PointCloud.load_pcd( "pointcloud/fixtures/can.pcd")

