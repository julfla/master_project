""" Tests definitions of identifier app. """

from django.test import TestCase
from pointcloud.models import PointCloud
from sketchup_models.models import SketchupModel
from identifier.models import Identifier
from system_evaluation.models import Example


class SimpleTest(TestCase):

    """ Test case for the app. """

    def test_exception_when_no_existing_category(self):
        """ Test that if the identifier is empty it throws. """
        pointcloud = PointCloud.load_pcd("pointcloud/fixtures/cloud.pcd")
        identifier = Identifier()
        self.assertRaises(IndexError, identifier.identify, pointcloud)
        # training will add a category :
        model = SketchupModel()
        model.google_id = "test1"
        model.mesh = file("sketchup_models/fixtures/mesh_can.tri").read()
        identifier.add_models([model], "test_category")
        try:
            identifier.identify(pointcloud)
        except IndexError:
            self.fail("identifier.identify() raised IndexError unexpectedly!")
        except:
            # can raise if Indentification failed
            pass

    from unittest import skip

    @skip("Need some preloading")
    # Run 'django preload_examples banana bowl food_can'
    def test_identification_banana_vs_bowl_vs_food_can(self):
        """ Try to identify with 3 categories. """
        # Getting the dataset
        bowl_ids = ['fa61e604661d4aa66658ecd96794a1cd',
                    'f74bba9a22e044dea3769fcd5f96f4',
                    'd2e1dc9ee02834c71621c7edb823fc53']
        banana_ids = ['f6e6117261dca163713c042b393cc65b',
                      'ba0d56295321002718ddbf38fa69c501',
                      '7d78e217e0ba160fe2b248b8bb97d290']
        bowls = []
        for bowl_id in bowl_ids:
            bowls.append(SketchupModel.find_google_id(bowl_id))
        bananas = []
        for banana_id in banana_ids:
            bananas.append(SketchupModel.find_google_id(banana_id))
        # Training
        iden = Identifier()
        iden.add_models(bananas, 'banana')
        iden.add_models(bowls, 'bowl')
        iden.train()
        # Identification
        for i in range(20):
            example = Example.get_random(['banana', 'bowl'])
            pcd_file = example.pcd_file()
            print "Identification of file {}".format(example)
            cloud = PointCloud.load_pcd(pcd_file.name)
            iden.identify(cloud)
