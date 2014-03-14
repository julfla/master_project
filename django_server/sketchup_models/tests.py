"""
This file demonstrates writing tests using the unittest module. These will pass
when you run "manage.py test".

Replace this with more appropriate tests for your application.
"""

from django.test import TestCase
from sketchup_models.models import SketchupModel


class SimpleTest(TestCase):
    def test_download_one_model(self):
        """"
        Test that a model is correctly download from the database.
        Link to the test model :
          https://3dwarehouse.sketchup.com/model.html?id=9fc96d41ec7a66a6a159545213d74ea
        """
        model_id = "9fc96d41ec7a66a6a159545213d74ea"
        model = SketchupModel._scrap_model_page(model_id)
        self.assertEqual(model.google_id, model_id)
        self.assertEqual(model.title, "Mug")
        self.assertEqual(model.text, "coffee mug or cup")
        tags = ["coffee", "coffee mug", "cup", "drink", "glass", "liquid", "mug"]
        self.assertEqual(model.tags, tags)

        # test the image and the mesh file from fixture
        tri_file = file("sketchup_models/fixtures/9fc96d41ec7a66a6a159545213d74ea.tri").read()
        img_file = file("sketchup_models/fixtures/9fc96d41ec7a66a6a159545213d74ea.jpg").read()
        self.assertEqual( model.mesh, tri_file)
        self.assertEqual( model.image, img_file)

    def test_listing_models_id_from_database(self):
        """
        Test that the search engine is used correctly and that we retreive a valid set of model
        """
        model_ids = SketchupModel._scrap_search_engine("mug")
        self.assertEqual( len(model_ids), 16)
        # get the right number of reference
        # all of them have a matching text/title/tags ?
