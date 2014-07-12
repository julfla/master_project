""" Define a Django Command for system evaluation. """

from django.core.management.base import BaseCommand
from optparse import make_option
import pickle
import json
import re

from sketchup_models.models import SketchupModel
from system_evaluation.models import ExampleObject
from identifier.models import Identifier
from sklearn.svm import LinearSVC, SVC
from sklearn.multiclass import (OneVsOneClassifier,
                                OneVsRestClassifier,
                                OutputCodeClassifier)
from sklearn.tree import DecisionTreeClassifier
from sklearn.neighbors import KNeighborsClassifier, RadiusNeighborsClassifier
from collections import defaultdict, OrderedDict
import operator

class Command(BaseCommand):

    """ Django Command for classification evaluation. """

    dataset = dict()
    identifier = None
    results = OrderedDict()
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
        'KNeighbors_default': KNeighborsClassifier(),
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
        if not self.identifier:
            if not self.dataset or options['force_dataset']:
                self.load_dataset(options)
            self.perform_learning(options['classifier_type'])
            # We also want to reidentify objects
            self.results = OrderedDict()
            self.done_examples = []
        if options['learning']:  # We stop after learning
            self.dump(options)
            return
        examples = ExampleObject.objects.filter(
            category__in=self.dataset.keys()).order_by("name")
        try:
            for index, example in enumerate(examples.iterator()):
                if example.pk not in self.done_examples:
                    print "Identification of {} {}/{} ({}%)".format(
                        example.name,
                        index + 1, examples.count(),
                        100 * (index + 1) / examples.count())
                    self.results[example.name] = self.process_example(example,
                                                                      options)
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
            if options['force_descriptors']:
                distribution = frame.get_distribution(True, True)
            else:
                distribution = frame.distribution
            result = self.identifier.identify(distribution)
            frame_results[result] += 1
        sorted_results = sorted(frame_results.iteritems(), reverse=True,
                                key=operator.itemgetter(1))
        return sorted_results[0][0]

    def results_by_category(self):
        from collections import OrderedDict
        category_m = re.compile("[a-z_]*[a-z]")
        categories = OrderedDict()
        for example, results in self.results.iteritems():
            example_category = category_m.match(example).group(0)
            if example_category not in categories:
                categories[example_category] = OrderedDict()
            categories[example_category][example] = results
        return categories

    def analyse_results(self):
        """ Analyze the results and print it in the terminal. """
        print "Results by category"
        for category, examples in self.results_by_category().iteritems():
            num_sequences = sum([len(seq) for seq in examples.values()])
            failures = []
            for example, sequences in examples.iteritems():
                for index, result in enumerate(sequences):
                    if category != result:
                        failures.append((example, index, result))
            num_positives = num_sequences - len(failures)
            print "{}: {}/{} ({}%)".format(category, num_positives,
                                           num_sequences,
                                           100 * num_positives / num_sequences)
            if failures:
                print "Failures :"
            for example, sequence, result in failures:
                print "     {} seq {} -> {}".format(example, sequence, result)

        # print "Results by object"
        # sorted_results = sorted(self.results.iteritems(),
        #                         key=operator.itemgetter(0))
        # for example_name, sequence_results in sorted_results:
        #     print "Result of %s" % example_name
        #     for index, sequence_result in enumerate(sequence_results):
        #         print "    sequence {}: {}".format(index+1, sequence_result)

        num_positives = 0
        num_sequences = 0
        for category, examples in self.results_by_category().iteritems():
            for examples, sequences in examples.iteritems():
                for result in sequences:
                    num_sequences += 1
                    if result == category:
                        num_positives += 1
        print "Overall result: {}%".format(100 * num_positives / num_sequences)
        print "    {} categories, {} objects".format(
            len(self.results_by_category()), len(self.results))

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
        state['identifier'] = self.identifier
        state['results'] = self.results
        with open(state_file_path, 'wb') as handle:
            pickle.dump(state, handle)
        print "State has been saved to {}.".format(state_file_path)

    def load(self, state_file_path):
        """ Load the process state to be restarted. """
        with open(state_file_path, 'r') as handle:
            state = pickle.load(handle)
        self.__dict__.update(state)
        print "State has been restored from {}.".format(state_file_path)
