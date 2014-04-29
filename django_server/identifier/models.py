from django.db import models
from djangotoolbox.fields import ListField, SetField
from django.utils.six import with_metaclass

from sketchup_models.models import SketchupModel
from shape_distribution.models import ShapeDistribution, SHAPE_DISTRIBUTION_SIZE
from partial_view.models import PartialView

from sklearn import svm
import numpy

class SVCField(with_metaclass(models.SubfieldBase, models.Field)):
    from sklearn.externals import joblib
    # Recreate python object from db
    def to_python(self, value):
        if isinstance(value, svm.classes.SVC):
            return value
        else:
            import tempfile
            with tempfile.NamedTemporaryFile() as f :
                f.write( value )
                f.flush()
                return joblib.load(f.name)

    # Serialize python object to be stored in db
    def get_prep_value(self, value):
        if isinstance(value, svm.classes.SVC):
            import tempfile
            with tempfile.NamedTemporaryFile() as f :
                joblib.dump(value, f.name)
                return f.read()
        else:
            return value

class Category(models.Model):
    name = models.CharField(unique=True, max_length=255)
    models = SetField(models.ForeignKey(SketchupModel) )

class Identifier(models.Model):

    categories = ListField()
    classifier = SVCField()
    
    @staticmethod
    def instance():
        try:
            return Identifier.objects.all()[0]
        except:
            return Identifier()
    
    def identify(self, pointcloud):
        """
        Returns the category of the pointcloud object if its category is known.
        If not, throws an exception.
        """
        if len( self.categories ) == 0: raise IndexError("Identifier is empty.")

        # TODO
        # first behaviour so that can be used in integration.
        distribution = ShapeDistribution(pointcloud)
        category_idx = self.classifier.predict(distribution.as_numpy_array)[0]
        print "Object identify as {}".format(categories[category_idx])

        from random import randint
        if randint(0, 1) == 0:
            raise Exception("Identification failed.")
        else:
            return self.categories[0]

    def train(self, models, category):
        """
        Incrementaly trains the classifier with the new model.
        If the category is not known yet, then it is added and returns true.
        """
        print "Adding models to category {}.".format( category )
        category, created = Category.objects.get_or_create(name=category)
        if created: print "The category {} has been created.".format(category.name)
        for model in models:
            category.models.add( model.pk )
        category.save()
        (X, Y) = self._get_example_matrix()
        print "Size X => {}     Size Y => {}".format(X.shape, Y.shape)
        self.classifier = svm.SVC(kernel='linear')
        print self.classfier.fit(X, Y)
        self.save()


    def _recompute_svm(self):
        """
        Recompute the SVM with all the category known
        """
        from sklearn import svm

    def _get_example_matrix(self):
        """
        Return the input matrix of example (X)
        """
        X = numpy.zeros( [0, SHAPE_DISTRIBUTION_SIZE] )
        Y = numpy.zeros( [0, 1] )
        for idx, category in enumerate(Category.objects.all()):
            arr = numpy.zeros( (0, SHAPE_DISTRIBUTION_SIZE ) )
            for model in SketchupModel.objects.filter(pk__in=category.models):
                if model.partialview_set.count() == 0:
                    PartialView.compute_all_views(model)
                for view in model.partialview_set.all():
                    arr = numpy.vstack( [arr, view.distribution.as_numpy_array] )
            X = numpy.vstack( [X, arr] )
            Y = numpy.vstack( [Y, idx * numpy.ones( [len(category.models) * 64, 1] ) ] )
        return (X, Y)
