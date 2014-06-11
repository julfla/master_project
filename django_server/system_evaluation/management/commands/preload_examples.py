""" Define a Django Command to seed the examples database. """

from django.core.management.base import BaseCommand
from optparse import make_option
from system_evaluation.models import Example

import tarfile
import re
import tempfile
import urllib
import json
import sys


def dowload_with_progress_bar(url):
    """ Display a progress bar while dowloading a file. """
    def reporthook(count, block_size, total_size):
        """ Print the bar. """
        progress_size = int(count * block_size)
        percent = int(count * block_size * 100 / total_size)
        sys.stdout.write(
            "\r...{}%, {}MB".format(percent, progress_size / (1024 * 1024))
            )
        sys.stdout.flush()
    print "Dowloading file {} :".format(url)
    temp_file = tempfile.NamedTemporaryFile()
    urllib.urlretrieve(url, temp_file.name, reporthook)
    print ''  # reporthook need a end line after using
    return temp_file


class Command(BaseCommand):

    """ Django Command to seed the example database from an online dataset. """

    help = ('Dowload some examples from an online dataset\n'
            'Usage is preload_models [options] CATEGORY1 CATEGORY2 ...')

    option_list = BaseCommand.option_list + (
        make_option('-l', '--list',
                    dest='list_categories',
                    action='store_true',
                    help='List the available categories.'),
        )

    def dowload_and_create_examples(self, example_object):
        # dowload image tar
        image_file = dowload_with_progress_bar(example_object['image_tar'])
        # extract example path without extension
        image_tar = tarfile.open(image_file.name)
        pcd_tar = None
        example_image_members = []
        # filter the archive content for the good image only
        m = re.compile("\S+_crop.png")
        for example_image_member in image_tar.getmembers():
            if m.search(example_image_member.name):
                example_image_members.append(example_image_member)
        # iterate on example path and create examples
        for example_image_member in example_image_members:
            # without the _crop.png
            example_path = re.sub('_crop.png', '', example_image_member.name)
            example = Example(name=example_path.split('/')[-1],
                              _compressed=True)
            # escape if object exists already
            if Example.objects.filter(name=example.name).exists():
                return
            print "Adding object {}".format(example.name)
            # we need to donwload the pcd tar now
            if pcd_tar is None:
                pcd_file = dowload_with_progress_bar(example_object['pcd_tar'])
                pcd_tar = tarfile.open(pcd_file.name)
            m = re.compile('{}.pcd'.format(example_path))
            for example_pcd_member in pcd_tar.getmembers():
                if m.match(example_pcd_member.name):
                    break
            example.pcd_file = pcd_tar.extractfile(example_pcd_member)
            example.image_file = image_tar.extractfile(example_image_member)
            example.save()

    def handle(self, *args, **options):
        """ Handle the command call. """
        available_examples = json.load(open('dataset.json'))
        if options['list_categories']:
            categories = set(Example(name=example['name']).category
                          for example in available_examples)
            for category in sorted(categories):
                print category
            return
        for category in args:
            for example_object in available_examples:
                if example_object['name'].startswith(category):
                    self.dowload_and_create_examples(example_object)
