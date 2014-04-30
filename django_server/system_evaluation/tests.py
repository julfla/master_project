from django.test import TestCase
from models import ExampleManager
import re, os

class TestSystemEvaluation(TestCase):

    def test_example_manager(self):
    	"""
    	Test that example manager is able to retreive some test models from the database.
    	"""
        manager = ExampleManager()
        example_name = manager.get_random_example()
        # test the foramt of the example_name
        self.assertEqual( example_name,
            re.match('[a-z_]+_[0-9]+_[0-9]+_[0-9]+', example_name).group(0) )
        img_file = manager.get_image( example_name )
        pcd_file = manager.get_pcd( example_name )
        # check that the files are not empty
        self.assertTrue( img_file.read() > 0 )
        self.assertTrue( pcd_file.read() > 0 )
        # test that the files correspond to an accessible path
        self.assertTrue( os.path.isfile( img_file.name ) )
        self.assertTrue( os.path.isfile( pcd_file.name ) )
        # test that the temporary file pcd_file is destroyed on closure
        pcd_path = pcd_file.name
        pcd_file = None
        self.assertTrue( not os.path.isfile( pcd_path ) )

    def test_restrict_to_some_categories(self):
        """
        Test that we can restrict the ful database of example to some categories only
        """
        manager = ExampleManager()
        for example in ExampleManager().list_examples( ['banana', 'bowl'] ):
            assertTrue( example.startswith('banana') or example.startswith('bowl'))