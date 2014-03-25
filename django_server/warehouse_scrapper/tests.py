from django.test import TestCase
from warehouse_scrapper.models import WarehouseScrapper


class SimpleTest(TestCase):
    def test_download_one_model(self):
        """"
        Test that a model is correctly download from the database.
        Link to the test model :
          https://3dwarehouse.sketchup.com/model.html?id=9fc96d41ec7a66a6a159545213d74ea
        """
        model_id = "9fc96d41ec7a66a6a159545213d74ea"
        model = WarehouseScrapper.scrap_one_model(model_id)
        self.assertEqual(model.google_id, model_id)
        self.assertEqual(model.title, "Mug")
        self.assertEqual(model.text, "coffee mug or cup")
        tags = ["coffee", "coffee mug", "cup", "drink", "glass", "liquid", "mug"]
        self.assertEqual(model.tags, tags)

        # test the image and the mesh file from fixture
        tri_file = file( "sketchup_models/fixtures/%s.tri" % model_id ).read()
        img_file = file( "sketchup_models/fixtures/%s.jpg" % model_id ).read()
        self.assertEqual( model.mesh, tri_file)
        self.assertEqual( model.image, img_file)

    def test_listing_models_id_from_database(self):
        """
        Test that the search engine is used correctly and 
        that we retreive a valid set of model
        """
        models = WarehouseScrapper.search_for_models("mug")
        self.assertEqual( len(models), 16)
        # get the right number of reference
        # all of them have a matching text/title/tags ?