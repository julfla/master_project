from django.db import models
from djangotoolbox.fields import DictField, SetField, EmbeddedModelField

from django.utils.six import with_metaclass

from sketchup_models.models import SketchupModel
from shape_distribution.models import ShapeDistribution, SHAPE_DISTRIBUTION_SIZE
from partial_view.models import PartialView

from sklearn import svm
import pickle
import numpy

class SVCField(with_metaclass(models.SubfieldBase, models.Field)):

    import inspect
    svm_classifier_classes = inspect.getmembers(svm.classes, inspect.isclass)
    svm_classifier_classes = map( lambda x: x[1], svm_classifier_classes)
    # Recreate python object from db
    def to_python(self, value):
        if value.__class__ in self.svm_classifier_classes:
            return value
        elif value is not None and len(value) > 0:
            return pickle.loads(value)
        return None
            
    # Serialize python object to be stored in db
    def get_prep_value(self, value):
        if value.__class__ in self.svm_classifier_classes:
            return pickle.dumps( value )
        else:
            return value

class Identifier(models.Model):

    classifier = SVCField(blank=True, null=True)
    dict_categories = DictField(SetField, default=dict())
    
    @staticmethod
    def instance():
        try:
            return Identifier.objects.all()[0]
        except:
            return Identifier()
    
    def identify_with_proba(self, pointcloud):
        """
        Returns the category of the pointcloud object if its category is known.
        If not, throws an exception.
        """
        if len( self.dict_categories ) < 1 :
            print "No category cannot identify"
            raise IndexError("Identifier is empty.")

        data = ShapeDistribution.compute(pointcloud).as_numpy_array
        result_proba = self.classifier.decision_function( data )
        result_idx = int(self.classifier.predict( data )[0])
        result_name = self.dict_categories.keys()[result_idx]
        return (result_name, result_proba)

    def identify(self, pointcloud):
        (result_name, result_proba) = self.identify_with_proba(pointcloud)
        return result_name

    def add_models(self, models, category_name):
        """
        Add models to the category.
        If the category is not known yet, it will be created.
        """
        if not category_name in self.dict_categories:
            self.dict_categories[category_name] = set()
            print "The category {} has been created.".format(category_name)
        category = self.dict_categories[category_name]
        print "Adding models to category {}.".format( category_name )
        for model in models:
            category.add( model.google_id )

    def train(self):
        """
        Train the classifier for all the models that it knows
        """
        if len( self.dict_categories ) < 2:
            print "At least two categories are needed for training..."
            print "Training is skipped."
        (X, Y) = self._get_example_matrix()
        print "Training with {} categories and {} views.".format( 
            len(self.dict_categories), len(Y)
            )
        self.classifier = svm.LinearSVC()
        print self.classifier.fit(X, Y)


    def _get_example_matrix(self):
        """
        Return the input matrix of example (X)
        """
        X = numpy.zeros( [0, SHAPE_DISTRIBUTION_SIZE] )
        Y = numpy.zeros( [0, 1] )
        for idx, (category, model_ids) in enumerate(self.dict_categories.items()):
            arr = numpy.zeros( (0, SHAPE_DISTRIBUTION_SIZE ) )
            for model_id in model_ids:
                model = SketchupModel.find_google_id(model_id)
                if model.partialview_set.count() == 0:
                    PartialView.compute_all_views(model)
                for view in model.partialview_set.all():
                    arr = numpy.vstack( [arr, view.distribution.as_numpy_array] )
            X = numpy.vstack( [X, arr] )
            Y = numpy.vstack( [Y, idx * numpy.ones( [len(model_ids) * 64, 1] ) ] )
        return (X, Y)
