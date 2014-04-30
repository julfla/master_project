from django.test import TestCase
from pointcloud.models import PointCloud
from sketchup_models.models import SketchupModel
from identifier.models import Identifier
from shape_distribution.models import ShapeDistribution
from system_evaluation.models import ExampleManager

class SimpleTest(TestCase):

    def test_exception_when_no_existing_category(self):
        pointcloud = PointCloud.load_pcd("pointcloud/fixtures/cloud.pcd")
        identifier = Identifier()
        self.assertRaises( IndexError, identifier.identify, pointcloud )
        # training will add a category :
        model = SketchupModel()
        model.google_id = "test1"
        model.mesh = file("sketchup_models/fixtures/mesh_can.tri").read()
        identifier.train( [model], "test_category" )
        try:
            identifier.identify( pointcloud )
        except IndexError:
            self.fail("identifier.identify() raised IndexError unexpectedly!")
        except:
            # can raise if Indentification failed
            pass

    def test_identification_banana_vs_bowl(self):
        bowl_ids = ['fa61e604661d4aa66658ecd96794a1cd',
            'f74bba9a22e044dea3769fcd5f96f4',
            'd2e1dc9ee02834c71621c7edb823fc53']
        banana_ids = ['f6e6117261dca163713c042b393cc65b',
            'ba0d56295321002718ddbf38fa69c501',
            '7d78e217e0ba160fe2b248b8bb97d290']
        bowls = []
        for bowl_id in bowl_ids:
            bowls.append( SketchupModel.find_google_id(bowl_id) )
        bananas = []
        for banana_id in banana_ids:
            bananas.append( SketchupModel.find_google_id(banana_id) )

        iden = Identifier()
        self.assertEquals( len(iden.categories), 0)
        iden.train( bananas, 'banana')
        self.assertEquals( len(iden.categories), 1)
        iden.train( bowls, 'bowl')
        self.assertEquals( len(iden.categories), 2)

        print "Identification of SketchupModels"
        import random
        print "Bananas"
        for banana in bananas:
            choice = random.choice( banana.partialview_set.all() )
            iden.identify( choice.pointcloud )
        print "Bowls"
        for bowl in bowls:
            choice = random.choice( bowl.partialview_set.all() )
            iden.identify( choice.pointcloud )


        print "Identification of real objects"
        for i in range(20):
            example = ExampleManager.get_random_example()
            pcd_file = ExampleManager.get_pcd( example )
            print "Identification of file {}".format( example )
            cloud = PointCloud.load_pcd( pcd_file.name )
            iden.identify( cloud )
