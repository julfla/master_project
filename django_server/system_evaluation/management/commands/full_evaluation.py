from django.core.management.base import BaseCommand, CommandError
from optparse import make_option
import pickle, json

from sketchup_models.models import SketchupModel
from system_evaluation.models import Example
from identifier.models import Identifier
from pointcloud.models import PointCloud

class Command(BaseCommand):
    help = ('Perform learning from a preset set of models,'
        + ' and test vs all bowls and bananas')

    option_list = BaseCommand.option_list + (
        make_option('--dataset',
            dest='dataset_file',
            default='evaluation_dataset.json.sample',
            help='Json file containing model ids of learnign dataset.'),
        make_option('--save',
            dest='save_file',
            default=None,
            help=('When interupted, the task state is store in this file,' + 
                ' so it can be resumed. Default to load parameter if used.')),
        make_option('--load',
            dest='load_file',
            default=None,
            help='Resume task from this file.'),
        make_option('--analyze',
            dest='analyze',
            action='store_true',
            default=False,
            help='Display results of evaluation.'),
        make_option('--learning-only',
            dest='learning',
            action='store_true',
            default=False,
            help='Performs the learning only, do not evaluate.'),
        )

    def handle(self, *args, **options):
        if options['load_file']:
            if not options['save_file']:
                options['save_file'] = options['load_file']
            self.load( options['load_file'] )
        else:
            self.pending_examples = None
            self.done_examples = None
            with open( options['dataset_file'] ) as f:
                self.dataset = json.load(f)
            self.perform_learning()
        if options['learning']:
            if options['save_file']:
                print 'Saving state into {}.'.format( options['save_file'] )
                self.dump( options['save_file'] )
            return
        if self.pending_examples is None and self.done_examples is None:
            self.initialize_list_examples()
        try:
            while len(self.pending_examples):
                example = self.pending_examples[0]
                self.process_example( example )
                self.done_examples.append( example )
                self.pending_examples.remove(example)
        except KeyboardInterrupt:            
            if options['save_file']:
                print 'Saving state into {}.'.format( options['save_file'] )
                self.dump( options['save_file'] )
            return
        if options['analyze']:
            self.analyse_results()
        if options['save_file']:
            print 'Saving state into {}.'.format( options['save_file'] )
            self.dump( options['save_file'] )

    def perform_learning(self):
        # Retreiving the dataset models
        models = {}
        for category in self.dataset.keys():
            models[category] = []
            for google_id in self.dataset[category]:
                models[category].append( SketchupModel.find_google_id(google_id) )
        # Training
        self.identifier = Identifier()
        for category in models.keys():
            self.identifier.train( models[category], category)

    def initialize_list_examples(self):
        self.pending_examples = []
        self.done_examples = []
        for category in self.dataset.keys():
            # this is super slow ??
            examples = map( lambda x: x.name, Example.filter_categories([category]) )
            for example in examples:
                self.pending_examples.append({'name': example, 'expected': category})

    def process_example(self, example):
        # Display of completion
        number_done = len(self.done_examples)
        number_total = number_done + len(self.pending_examples)
        print 'Dealing with {} ( {}/{}, {}% )'.format( example['name'],
            number_done + 1, number_total, 100 * number_done / number_total )
        pcd_file = Example.objects.get(name=example['name']).pcd_file
        cloud = PointCloud.load_pcd( pcd_file.name )
        (category, proba) = self.identifier.identify_with_proba(cloud)
        example['actual'] = category
        example['proba'] = proba
        # display of results
        print '{} has been identified has a {}, {}'.format(
            example['name'], example['actual'], example['proba'] )

    def analyse_results(self):
        examples = self.done_examples
        good_results = []
        for example in examples:
            if example['actual'] == example['expected']:
                good_results.append(example)
        print "Identification of {} examples.".format( len(examples) )
        print "Good results : {}%".format(100 * len(good_results)/len(examples))
        # add per object rate and per category object
        # display list of unsuccessfull classification with ordered proba results

    def dump(self, state_file_path):
        state = {}
        state['dataset'] = self.dataset
        state['done_examples'] = self.done_examples
        state['pending_examples'] = self.pending_examples
        state['identifier'] = self.identifier
        with open(state_file_path, 'wb') as handle:
            pickle.dump( state, handle )
        print "State has been saved to {}.".format( state_file_path )

    def load(self, state_file_path):
        with open(state_file_path, 'r') as handle:
            state = pickle.load( handle )
        self.__dict__.update( state )
        print "State has been restored from {}.".format( state_file_path )
