""" Module for pointcloud identification. """

from django.db import models
from djangotoolbox.fields import DictField, SetField

from django.utils.six import with_metaclass

from sketchup_models.models import SketchupModel
from shape_distribution.models import (ShapeDistribution,
                                       SHAPE_DISTRIBUTION_SIZE)
from partial_view.models import PartialView

from sklearn import svm, multiclass
import pickle
import numpy


class SVCField(with_metaclass(models.SubfieldBase, models.Field)):

    """ A django field conresponding to sklearn classifier objects. """

    import inspect
    svm_classifier_classes = inspect.getmembers(svm, inspect.isclass)
    svm_classifier_classes += inspect.getmembers(multiclass, inspect.isclass)
    svm_classifier_classes = (x[1] for x in svm_classifier_classes)

    def to_python(self, value):
        """ Recreate python object from db. """
        if value.__class__ in self.svm_classifier_classes:
            return value
        elif value is not None and len(value) > 0:
            return pickle.loads(value)
        return None

    def get_prep_value(self, value):
        """ Serialize python object to be stored in db. """
        if value.__class__ in self.svm_classifier_classes:
            return pickle.dumps(value)
        else:
            return value


class Identifier(models.Model):

    """ Identifier for pointcloud objects. """

    classifier = SVCField()
    dict_categories = DictField(SetField, default=dict())

    def identify_with_proba(self, data):
        """ Return the category of the Pointcloud or Distribution object. """
        if len(self.dict_categories) < 1:
            print "No category cannot identify"
            raise IndexError("Identifier is empty.")
        if type(data) == 'PointCloud':
            data = ShapeDistribution.compute(data)
        if type(data) == 'ShapeDistribution':
            data = data.as_numpy_array
        result_proba = None
        try:
            result_proba = self.classifier.decision_function(data)
        except AttributeError:
            pass
        result_idx = int(self.classifier.predict(data)[0])
        result_name = self.dict_categories.keys()[result_idx]
        return (result_name, result_proba)

    def identify(self, data):
        """ Return only the identification result, no probability. """
        (result_name, _) = self.identify_with_proba(data)
        return result_name

    def add_models(self, models, category_name):
        """ Add models to the category, create the category if necessary. """
        if category_name not in self.dict_categories:
            self.dict_categories[category_name] = set()
            print "The category {} has been created.".format(category_name)
        category = self.dict_categories[category_name]
        print "Adding models to category {}.".format(category_name)
        for model in models:
            category.add(model.google_id)

    def train(self):
        """ Train the classifier for all the models that it knows. """
        if len(self.dict_categories) < 2:
            print "At least two categories are needed for training..."
            print "Training is skipped."
        (X, Y, W) = self._get_example_matrix()
        print "Training with {} categories and {} views.".format(
            len(self.dict_categories), len(Y))
        print self.classifier.fit(X, Y)  # sample_weight=W)


    def _get_model_example_matrix(self, model):
        views = model.partialview_set.all()
        w = []
        # x = numpy.zeros([0, SHAPE_DISTRIBUTION_SIZE])
        x = numpy.vstack([v.distribution.as_numpy_array for v in views])
        w = [v.entropy for v in views]
        # Scale entropy so that each model has the same weight
        # max_w = max(w)
        mean_w = sum(w) / len(w)
        w = numpy.array([value / mean_w for value in w])
        return (x, w)

    def _get_example_matrix(self):
        """ Return the input matrix of example (X). """
        X = numpy.zeros([0, SHAPE_DISTRIBUTION_SIZE])
        Y = numpy.array([])
        W = numpy.array([])  # Weights of the samples
        for idx, (category, model_ids) in (enumerate(
                                           self.dict_categories.items())):
            for model_id in model_ids:
                model = SketchupModel.find_google_id(model_id)
                if model.partialview_set.count() == 0:
                    PartialView.compute_all_views(model)
                (x, w) = self._get_model_example_matrix(model)
                y = numpy.array([idx for _ in range(x.shape[0])])
                X = numpy.vstack([X, x])
                Y = numpy.concatenate([Y, y])
                W = numpy.concatenate([W, w])
        return (X, Y, W)
