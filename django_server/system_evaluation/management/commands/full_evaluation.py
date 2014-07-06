""" Define a Django Command for system evaluation. """

from django.core.management.base import BaseCommand
from optparse import make_option
import pickle
import json

from sketchup_models.models import SketchupModel
from system_evaluation.models import ExampleObject
from identifier.models import Identifier
from sklearn.svm import LinearSVC, SVC
from sklearn.multiclass import (OneVsOneClassifier,
                                OneVsRestClassifier,
                                OutputCodeClassifier)
from sklearn.tree import DecisionTreeClassifier
from sklearn.neighbors import KNeighborsClassifier, RadiusNeighborsClassifier
from collections import defaultdict
import operator

class Command(BaseCommand):

    """ Django Command for classification evaluation. """

    dataset = dict()
    identifier = None
    results = None
    done_examples = []

    help = ('Perform learning from a set of models,'
            ' and test it vs the dataset.')

    classifiers_available = {
        'LinearSVC': LinearSVC(random_state=0),
        'SVC': SVC(),
        'OneVsOne': OneVsOneClassifier(LinearSVC(random_state=0)),
        'OneVsRest': OneVsRestClassifier(LinearSVC(random_state=0)),
        'DecisionTree': DecisionTreeClassifier(),
        'KNeighbors': KNeighborsClassifier(weights='distance',
                                           n_neighbors=5,
                                           leaf_size=64),
        'RadiusNeighbors': RadiusNeighborsClassifier(weights='distance',
                                                     n_neighbors=5,
                                                     leaf_size=30,
                                                     radius=10.0),
        'OutputCode': OutputCodeClassifier(LinearSVC(random_state=0)),
        }

    option_list = BaseCommand.option_list + (
        make_option('-d', '--dataset',
                    dest='dataset_file',
                    default='evaluation_dataset.json.sample',
                    help='Json file with model ids of learnign dataset.'),
        make_option('-r', '--restrict',
                    dest='categories',
                    default=None,
                    help=('Restrict the dataset to a list of category.'
                          ' Separate categories with comma.')),
        make_option('-o', '--save',
                    dest='save_file',
                    default=None,
                    help=('When interupted, the task state is store in this '
                          'file, so it can be resumed. '
                          'Default to load parameter if used.')),
        make_option('-i', '--load',
                    dest='load_file',
                    default=None,
                    help='Resume task from this file.'),
        make_option('-a', '--analyze',
                    dest='analyze',
                    action='store_true',
                    default=False,
                    help='Display results of evaluation.'),
        make_option('-l', '--learning-only',
                    dest='learning',
                    action='store_true',
                    default=False,
                    help='Performs the learning only, do not evaluate.'),
        make_option('-c', '--classifier',
                    dest='classifier_type',
                    default='LinearSVC',
                    help='Kind of classifier to use.'),
        make_option('-f', '--force-descriptors',
                    dest='force_descriptors',
                    action='store_true',
                    default=False,
                    help="Force recomputation of descriptors."),
        )

    def handle(self, *_, **options):
        """ Handle the command call. """
        if options['load_file']:
            self.load(options['load_file'])
        if not self.identifier or options['force_learning']:
            if not self.dataset or options['force_dataset']:
                self.load_dataset(options)
            self.perform_learning(options['classifier_type'])
            # We also want to reidentify objects
            self.results = None
            self.done_examples = []
        if options['learning']:  # We stop after learning
            self.dump(options)
            return
        examples = ExampleObject.objects.filter(
            category__in=self.dataset.keys())
        try:
            for index, example in enumerate(examples.iterator()):
                if example.pk not in self.done_examples:
                    print "Identification of {} {}/{} ({}%)".format(
                        example.name,
                        index, examples.count(),
                        100 * index / examples.count())
                    print self.process_example(example, options)
                    self.done_examples.append(example.pk)
            if options['analyze']:
                self.analyse_results()
        # The process can be stopped and saved for restart
        except KeyboardInterrupt:
            pass
        self.dump(options)

    def load_dataset(self, options):
        """ Load the dataset according to the options. """
        with open(options['dataset_file']) as dataset_file:
            self.dataset = json.load(dataset_file)
        if options['categories']:  # We restrict the dataset to some categories
            categories = options['categories'].split(',')
            for category in categories:
                if category not in self.dataset:
                    raise Exception("{} is not included in the dataset".
                                    format(category))
            self.dataset = {k: self.dataset[k] for k in categories}

    def perform_learning(self, classifier_type):
        """ Perfrom the learning from the dataset. """
        # Retreiving the dataset models
        print 'Training using a {} classifier.'.format(classifier_type)
        models = {}
        for category in self.dataset.keys():
            models[category] = []
            for google_id in self.dataset[category]:
                models[category].append(
                    SketchupModel.find_google_id(google_id))
        # Training
        classifier = self.classifiers_available[classifier_type]
        self.identifier = Identifier(classifier=classifier)
        for category in models.keys():
            self.identifier.add_models(models[category], category)
        # (x, y, w) = self.identifier._get_example_matrix()
        # import matplotlib.pyplot as plt
        # import numpy as np
        # # w = np.array(w)
        # # print y
        # # print w
        # plt.plot(y, w, 'ro')
        # print self.dataset.keys()
        # plt.show()
        self.identifier.train()

    def process_example(self, example, options):
        """ Identify an example object and return the result. """
        results = []
        for sequence in example.sequences.all():
            results.append(self.process_videosequence(sequence, options))
        return results

    def process_videosequence(self, video_sequence, options):
        """ Perform the identification for a sequence.

        This should be in the Identifier class maybe...
        """
        frame_results = defaultdict(int)
        for frame in video_sequence.frames.iterator():
            result = self.identifier.identify(frame.distribution)
            frame_results[result] += 1
        sorted_results = sorted(frame_results.iteritems(), reverse=True,
                                key=operator.itemgetter(1))
        return sorted_results[0][0]


    def analyse_results(self):
        """ Analyze the results and print it in the terminal. """
        def result_group_by(examples, key, displayed_name=None):
            """ Group example by the value of the key provided. """
            if displayed_name is None:
                displayed_name = key
            results = defaultdict(lambda: defaultdict(list))
            for example in examples:
                key_value = example[key]
                results[key_value]['all'].append(example)
                if example['expected'] == example['actual']:
                    results[key_value]['positives'].append(example)
                else:
                    results[key_value]['negatives'].append(example)
            for group, item in results.items():
                results[group]['percentage'] = (100 * len(item['positives']) /
                                                len(item['all']))

            sorted_results = sorted(results.items(),
                                    key=lambda x: x[1]['percentage'])
            print 'Results by {}'.format(displayed_name)
            for group, item in sorted_results:
                error_stats = defaultdict(int)
                for example in item['negatives']:
                    error_stats[example['actual']] += 1
                sorted_error_starts = sorted(error_stats.iteritems(),
                                             key=lambda x: x[1], reverse=True)
                detail_result = ', '.join(
                    ["{}: {}".format(x[0], x[1]) for x in sorted_error_starts])

                print '{} : {}% (total: {}) | Errors : {}'.format(
                    group, item['percentage'], len(item['all']), detail_result)
            print ''

        examples = self.done_examples
        positives = [a for a in examples if a['expected'] == a['actual']]
        print "Identification of {} examples.".format(len(examples))
        print "Overall results : {}%".format(
            100 * len(positives)/len(examples))
        print ''
        result_group_by(examples, 'expected', 'category')
        # result_group_by(examples, 'object_name', 'object')
        object_name_set = set([example['object_name']
                               for example in self.done_examples])
        positives = 0
        total = len(object_name_set)
        for object_name in sorted(object_name_set):
            results_count = defaultdict(int)
            expected = None
            for example in [example for example in self.done_examples
                            if example['object_name'] == object_name]:
                if not expected:
                    expected = example['expected']
                results_count[example['actual']] += 1
            result = max(results_count.iteritems(),
                         key=operator.itemgetter(1))[0]
            if expected == result:
                positives += 1
            else:
                print "{} is classified as a {}".format(object_name, result)
        print positives, " / ", total, " (", 100 * positives / total, "%)"

    def dump(self, options):
        """ Dump the process in a homemade format. """
        if options['save_file']:
            state_file_path = options['save_file']
        elif options['load_file']:
            state_file_path = options['load_file']
        else:
            return
        state = {}
        state['dataset'] = self.dataset
        state['done_examples'] = self.done_examples
        state['pending_examples'] = self.pending_examples
        state['identifier'] = self.identifier
        with open(state_file_path, 'wb') as handle:
            pickle.dump(state, handle)
        print "State has been saved to {}.".format(state_file_path)

    def load(self, state_file_path):
        """ Load the process state to be restarted. """
        with open(state_file_path, 'r') as handle:
            state = pickle.load(handle)
        self.__dict__.update(state)
        print "State has been restored from {}.".format(state_file_path)
