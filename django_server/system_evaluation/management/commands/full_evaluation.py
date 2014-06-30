""" Define a Django Command for system evaluation. """

from django.core.management.base import BaseCommand
from optparse import make_option
import pickle
import json

from shape_distribution.models import ShapeDistribution
from sketchup_models.models import SketchupModel
from system_evaluation.models import Example
from identifier.models import Identifier
from pointcloud.models import PointCloud
from sklearn.svm import LinearSVC, SVC
from sklearn.multiclass import (OneVsOneClassifier,
                                OneVsRestClassifier,
                                OutputCodeClassifier)
from sklearn.tree import DecisionTreeClassifier
from sklearn.neighbors import KNeighborsClassifier, RadiusNeighborsClassifier
from collections import defaultdict
import operator
from django_server.settings import MEDIA_ROOT

class Command(BaseCommand):

    """ Django Command for classification evaluation. """

    dataset = dict()
    identifier = None
    pending_examples = []
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
        make_option('-k', '--keep-descriptors',
                    dest='keep-descriptors',
                    action='store_true',
                    default=False,
                    help='Keep the descritors value in the output file.'),
        make_option('--force-dataset',
                    dest='force_dataset',
                    action='store_true',
                    default=False,
                    help="Force loading from dataset."),
        make_option('--force-learning',
                    dest='force_learning',
                    action='store_true',
                    default=False,
                    help="Force relearning."),
        make_option('--force-descriptors',
                    dest='force_descriptors',
                    action='store_true',
                    default=False,
                    help="Force recomputation of descriptors."),
        )

    def handle(self, *_, **options):
        """ Handle the command call. """
        if options['load_file']:
            self.load(options['load_file'])
        if not self.dataset or options['force_dataset']:
            self.load_dataset(options)
        if not self.identifier or options['force_learning']:
            self.perform_learning(options['classifier_type'])
            # We also want to reidentify objects
            self.pending_examples = self.done_examples
            self.done_examples = []
        if options['learning']:  # We stop after learning
            self.dump(options)
            return
        if not self.pending_examples and not self.done_examples:
            self.initialize_list_examples()
        try:
            while len(self.pending_examples):
                self.process_example(self.pending_examples[0], options)
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

    def initialize_list_examples(self):
        """ Generate the pending examples list from the the database. """
        self.pending_examples = []
        self.done_examples = []
        categories = self.dataset.keys()
        map_f = lambda example: {'name': example.name,
                                 'object_name': example.object_name,
                                 'expected': example.category}
        for example in Example.filter_categories(categories).iterator():
            self.pending_examples.append(map_f(example))

    def process_example(self, example, options):
        """ Process one example, and remove it from the pending list. """
        # Display of completion
        # if example['expected'] == 'food_can':
        #     example['expected'] = 'soda_can'
        number_done = len(self.done_examples)
        number_total = number_done + len(self.pending_examples)
        # print 'Dealing with {} ( {}/{}, {}% )'.format(
        #     example['name'],
        #     number_done + 1,
        #     number_total,
        #     100 * number_done / number_total)
        if options['force_descriptors']:
            # Remove the shape distribution if if was kept.
            example.pop('shape_distribution')
        if 'shape_distribution' not in example:
            pcd_path = Example.objects.get(name=example['name']).pcd.name
            pcd_path = str(MEDIA_ROOT + pcd_path)
            cloud = PointCloud.load_pcd(pcd_path)
            distribution = ShapeDistribution.compute(cloud).as_numpy_array
        else:
            distribution = example['shape_distribution']
        (category, proba) = self.identifier.identify_with_proba(distribution)
        example['actual'] = category
        example['proba'] = proba
        #  we may include the descriptors to the outputs
        if 'keep-descriptors' in options:
            example['shape_distribution'] = distribution
        # display of results
        # print '    {} has been identified as a {}, {}'.format(
        #     example['name'], example['actual'], example['proba'])
        self.done_examples.append(example)
        self.pending_examples.remove(example)

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
