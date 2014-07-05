from django.test import TestCase
from system_evaluation.models import Example


class TestSystemEvaluation(TestCase):

    def test_compressed_gridfsfield(self):
        example = Example(name='test', _compressed=True)
        example.image_file = 'hello'
        image_file = example.image_file
        self.assertEqual( image_file.read(), 'hello')

    def test_form_AgreeWithIdentificationForm_validation(self):
        from system_evaluation.forms import AgreeWithIdentificationForm
        # test that user_identification can be null if and only if user disagree
        form = AgreeWithIdentificationForm(data={'user_agreed': True})
        self.assertTrue(form.is_valid())
        form = AgreeWithIdentificationForm(data={'user_agreed': False})
        self.assertFalse(form.is_valid())
        form = AgreeWithIdentificationForm(data={'user_agreed': False, 'user_identification': 'choice'})
        self.assertTrue(form.is_valid())

    def test_example_regex(self):
        """ Test the regex used to retreive info from the example name. """
        example = Example(name='banana_2_1_166')
        self.assertEqual(example.category, "banana")
        self.assertEqual(example.object_name, "banana_2")

        example = Example(name='soda_can_2_1_1')
        self.assertEqual(example.category, "soda_can")
        self.assertEqual(example.object_name, "soda_can_2")
