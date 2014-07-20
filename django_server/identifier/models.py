""" Module for pointcloud identification. """

from django.db import models
from djangotoolbox.fields import DictField, SetField

from django.utils.six import with_metaclass

from sketchup_models.models import SketchupModel
from shape_distribution.models import (ShapeDistribution,
                                       SHAPE_DISTRIBUTION_SIZE)
from pointcloud.models import PointCloud
from partial_view.models import PartialView

from sklearn import covariance
import pickle
import numpy


class ClassifierField(with_metaclass(models.SubfieldBase, models.Field)):

    """ A django field conresponding to sklearn classifier objects. """

    def to_python(self, value):
        """ Recreate python object from db. """
        if hasattr(value, "fit"):
            return value
        elif value and len(value) > 0:
            return pickle.loads(value)
        return None

    def get_prep_value(self, value):
        """ Serialize python object to be stored in db. """
        if hasattr(value, "fit"):
            return pickle.dumps(value)
        else:
            return value


class Identifier(models.Model):

    """ Identifier for pointcloud objects. """

    classifier = ClassifierField()
    dict_categories = DictField(SetField, default=dict())

    def identify_with_proba(self, data):
        """ Return the category of the Pointcloud or Distribution object. """
        if len(self.dict_categories) < 2:
            print "No enough categories cannot identify"
            raise IndexError("Identifier is empty.")
        if type(data) is PointCloud:
            data = ShapeDistribution.compute(data)
        if type(data) is ShapeDistribution:
            data = data.as_numpy_array
        result_proba = None
        try:
            result_proba = self.classifier.decision_function(data)
        except AttributeError:
            pass
        try:
            result_idx = int(self.classifier.predict(data)[0])
            result_name = self.dict_categories.keys()[result_idx]
        except ValueError:
            result_name = 'Unknown'
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

    def train(self, use_entropy=False):
        """ Train the classifier for all the models that it knows. """
        if len(self.dict_categories) < 2:
            print "At least two categories are needed for training..."
            print "Training is skipped."
            return
        (X, Y, W) = self._get_example_matrix(use_entropy)
        if self.classifier.metric == 'mahalanobis':
            # The mahalanobis distance needs the covariance of the data
            cov = covariance.empirical_covariance(X)
            self.classifier.metric_kwds['V'] = cov
        print "Training with {} categories and {} views.".format(
            len(self.dict_categories), len(Y))
        print self.classifier.fit(X, Y)

    def _get_model_example_matrix(self, model, use_entropy):
        views = model.partialview_set
        if use_entropy:
            agg = views.aggregate(models.Avg('entropy'))
            views = views.filter(entropy__gt=agg['entropy__avg'])
        w = numpy.array([v.entropy for v in views.all()])
        x = numpy.array([v.distribution.as_numpy_array for v in views.all()])
        return (x, w)

    def _get_example_matrix(self, use_entropy):
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
                (x, w) = self._get_model_example_matrix(model, use_entropy)
                y = numpy.array([idx for _ in range(x.shape[0])])
                X = numpy.vstack([X, x])
                Y = numpy.concatenate([Y, y])
                W = numpy.concatenate([W, w])
        return (X, Y, W)
