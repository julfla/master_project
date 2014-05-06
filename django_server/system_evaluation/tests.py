from django.test import TestCase
from models import ExampleManager
import re, os
import forms

class TestSystemEvaluation(TestCase):

    def test_example_manager(self):
    	"""
    	Test that example manager is able to retreive some test models from the database.
    	"""
        example_name = ExampleManager.get_random_example()
        # test the foramt of the example_name
        self.assertEqual( example_name,
            re.match('[a-z_]+_[0-9]+_[0-9]+_[0-9]+', example_name).group(0) )
        img_file = ExampleManager.get_image( example_name )
        pcd_file = ExampleManager.get_pcd( example_name )
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

        def contain_examples_and_example_correct(categories):
            examples = ExampleManager.list_examples( categories )
            self.assertTrue( len(examples) > 0 )
            for example in examples:
                for category in categories:
                    if example.startswith(category): return
                self.fail("example {} does not start with one of {}".format(example, categories))

        # test for only one
        contain_examples_and_example_correct( ['banana'] )
        # test for two
        contain_examples_and_example_correct( ['banana', 'bowl'] )
        # test for three
        contain_examples_and_example_correct( ['banana', 'bowl', 'sponge'] )
        # test for composed word
        contain_examples_and_example_correct( ['food_can'] )

    def test_form_AgreeWithIdentificationForm_validation(self):
        from system_evaluation.forms import AgreeWithIdentificationForm
        # test that user_identification can be null if and only if user disagree
        form = AgreeWithIdentificationForm(data={'user_agreed': True})
        self.assertTrue(form.is_valid())
        form = AgreeWithIdentificationForm(data={'user_agreed': False})
        self.assertFalse(form.is_valid())
        form = AgreeWithIdentificationForm(data={'user_agreed': False, 'user_identification': 'choice'})
        self.assertTrue(form.is_valid())