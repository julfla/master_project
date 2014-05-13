from django.db import models
from djangotoolbox.fields import ListField, SetField, EmbeddedModelField

from django.utils.six import with_metaclass

from sketchup_models.models import SketchupModel
from shape_distribution.models import ShapeDistribution, SHAPE_DISTRIBUTION_SIZE
from partial_view.models import PartialView

from sklearn import svm
import pickle
import numpy

class SVCField(with_metaclass(models.SubfieldBase, models.Field)):
    # Recreate python object from db
    def to_python(self, value):
        if isinstance(value, svm.classes.SVC):
            return value
        elif value is not None and len(value) > 0:
            return pickle.loads(value)
        return None
            
    # Serialize python object to be stored in db
    def get_prep_value(self, value):
        if isinstance(value, svm.classes.SVC):
            return pickle.dumps( value )
        else:
            return value

class Identifier(models.Model):

    classifier = SVCField(blank=True, null=True)

    @property
    def categories(self):
        return Category.objects.filter(identifier=self).order_by('name')
    
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
        if len( self.categories ) < 1 :
            print "No category cannot identify"
            raise IndexError("Identifier is empty.")

        # TODO
        # first behaviour so that can be used in integration.
        data = ShapeDistribution.compute(pointcloud).as_numpy_array
        result_proba = self.classifier.predict_proba( data )
        result_idx = int(self.classifier.predict( data )[0])
        result_name = self.categories[result_idx].name
        return (result_name, result_proba)

    def identify(self, pointcloud):
        (result_name, result_proba) = self.identify_with_proba(pointcloud)
        return result_name

    def train(self, models, category_name):
        """
        Incrementaly trains the classifier with the new model.
        If the category is not known yet, then it is added and returns true.
        """
        # retreiving or creating the corresponding category
        self.save() # make sure that we have a primary key
        category, created = Category.objects.get_or_create(identifier=self, name=category_name)
        if created: print "The category {} has been created.".format(category.name)
        
        print "Adding models to category {}.".format( category_name )
        for model in models:
            category.models.add( model.pk )
        category.save()

        # training the classifier
        # cannot train with less than two classes
        if len( self.categories ) >= 2:
            (X, Y) = self._get_example_matrix()
            print "Size X => {}     Size Y => {}".format(X.shape, Y.shape)
            self.classifier = svm.SVC(kernel='linear', probability=True)
            print self.classifier.fit(X, Y)
        self.save()

    def _get_example_matrix(self):
        """
        Return the input matrix of example (X)
        """
        X = numpy.zeros( [0, SHAPE_DISTRIBUTION_SIZE] )
        Y = numpy.zeros( [0, 1] )
        for idx, category in enumerate(self.categories):
            arr = numpy.zeros( (0, SHAPE_DISTRIBUTION_SIZE ) )
            for model in SketchupModel.objects.filter(pk__in=category.models):
                if model.partialview_set.count() == 0:
                    PartialView.compute_all_views(model)
                for view in model.partialview_set.all():
                    arr = numpy.vstack( [arr, view.distribution.as_numpy_array] )
            X = numpy.vstack( [X, arr] )
            Y = numpy.vstack( [Y, idx * numpy.ones( [len(category.models) * 64, 1] ) ] )
        return (X, Y)

class Category(models.Model):
    name = models.CharField(max_length=255)
    identifier = models.ForeignKey(Identifier)
    models = SetField(models.ForeignKey( SketchupModel ))

    class Meta:
        unique_together = (("name", "identifier"),)
