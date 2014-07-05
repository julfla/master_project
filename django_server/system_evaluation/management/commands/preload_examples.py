""" Define a Django Command to seed the examples database. """

from django.core.management.base import BaseCommand
from django.core.files import File
from optparse import make_option
from system_evaluation.models import ExampleObject, VideoSequence, Frame
from django_server.settings import MEDIA_ROOT

import tarfile
import re
import tempfile
import urllib
import json
import sys
from os.path import basename, splitext


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


def preload_example_object(example_object):
    # Dowload the pcd archve file if needed
    if not example_object.pcd_tar:
        temp_file = dowload_with_progress_bar(example_object.url_pcd_tar)
        file_name = "%s_pcd.tar" % example_object.name
        example_object.pcd_tar.save(file_name, File(temp_file))
    # Dowload the image archve file if needed
    if not example_object.image_tar:
        temp_file = dowload_with_progress_bar(example_object.url_image_tar)
        file_name = "%s_image.tar" % example_object.name
        example_object.image_tar.save(file_name, File(temp_file))
    pcd_tar = tarfile.open(MEDIA_ROOT + example_object.pcd_tar.name)
    image_tar = tarfile.open(MEDIA_ROOT + example_object.image_tar.name)
    # We iterate over all example's frames
    for pcd_member in pcd_tar.getmembers():
        m = re.compile(pcd_member.name.replace('.pcd', '_crop.png'))
        for image_member in image_tar.getmembers():
            # If we found the corresponding member then we stop iterating
            if m.match(image_member.name):
                break
        # The path end with <...>_<sequence_id>_<frame_id>.pcd
        frame_number = pcd_member.name.split('_')[-1].split('.')[0]
        sequence_number = pcd_member.name.split('_')[-2]
        sequence, _ = VideoSequence.objects.get_or_create(
            example_object=example_object, sequence_id=sequence_number)
        Frame.objects.create(video_sequence=sequence,
                             frame_id=frame_number)


class Command(BaseCommand):

    """ Django Command to dowload the dataset of example objects. """

    help = ('Dowload some examples from an online dataset\n'
            'Usage is preload_models [options] CATEGORY1 CATEGORY2 ...')

    option_list = BaseCommand.option_list + (
        make_option('-l', '--list',
                    dest='list_categories',
                    action='store_true',
                    help='List the available categories.'),
        )

    def handle(self, *args, **options):
        """ Handle the command call. """
        if options['list_categories']:
            categories = set(example.category for example in
                             ExampleObject.objects.all())
            for category in sorted(categories):
                print category
            return
        for category in args:
            example_objects = ExampleObject.objects.filter(category=category)
            for example_object in example_objects:
                preload_example_object(example_object)
