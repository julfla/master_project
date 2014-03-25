from django.test import TestCase
from models import ExampleManager

class TestSystemEvaluation(TestCase):

    def test_example_manager(self):
    	"""
    	Test that example manager is able to retreive some test models from the database.
    	"""
        manager = ExampleManager()
        self.assertTrue(manager.size() > 0)
        res = manager.get_random_model()
        open("tmp/test.pcd", 'w+').write(res['pcd'].read())
        open("tmp/test.png", 'w+').write(res['img'].read())
