from django.test import TestCase
from pointcloud.models import PointCloud
from identifier.models import Identifier

class SimpleTest(TestCase):

    def test_exception_when_no_existing_category(self):
        pointcloud = PointCloud.load_pcd("pointcloud/fixtures/cloud.pcd")
        identifier = Identifier.instance()
        self.assertRaises( IndexError, identifier.identify, pointcloud )
        identifier.categories.append( "banana" )
        try:
            identifier.identify( pointcloud )
        except IndexError:
            self.fail("identifier.identify() raised IndexError unexpectedly!")

