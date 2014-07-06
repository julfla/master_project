""" Define a Django Command to seed the examples database. """

from django.core.management.base import BaseCommand
from django.core.files import File
from optparse import make_option
from system_evaluation.models import ExampleObject, VideoSequence, Frame
from pointcloud.models import PointCloud
from shape_distribution.models import ShapeDistribution
from django_server.settings import MEDIA_ROOT

import tarfile
import re
import tempfile
import urllib
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


def preload_example_object(example_object, options):
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
    # If we compute distributions or pointclouds
    # it would be faster to extract all the archive at once
    temp_dir = None
    if options['save_distribution'] or options['save_pointcloud']:
        temp_dir = tempfile.mkdtemp()
        pcd_tar.extractall(temp_dir)
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
        frame = Frame(video_sequence=sequence, frame_id=frame_number,
                      pcd_member_name=pcd_member.name,
                      image_member_name=image_member.name)
        if options['save_distribution'] or options['save_pointcloud']:
            pcd_path = "{}/{}".format(temp_dir, frame.pcd_member_name)
            pointcloud = PointCloud.load_pcd(pcd_path)
            if options['save_pointcloud']:
                frame._pointcloud = pointcloud
            if options['save_distribution']:
                frame._distribution = ShapeDistribution.compute(pointcloud)
        frame.save()
    if temp_dir:
        import shutil
        shutil.rmtree(temp_dir)


class Command(BaseCommand):

    """ Django Command to dowload the dataset of example objects. """

    help = ('Dowload some examples from an online dataset\n'
            'Usage is preload_models [options] CATEGORY1 CATEGORY2 ...')

    option_list = BaseCommand.option_list + (
        make_option('-l', '--list',
                    dest='list_categories',
                    action='store_true',
                    help='List the available categories.'),
        make_option('-a', '--all',
                    dest='preload_all',
                    action='store_true',
                    help='Download all the dataset.'),
        make_option('-p', '--save_pointcloud',
                    dest='save_pointcloud',
                    action='store_true',
                    help='Will also save the pointclouds in db.'),
        make_option('-d', '--save_distribution',
                    dest='save_distribution',
                    action='store_true',
                    help='Will also save the ShapeDistribution in db.'),
        )

    def handle(self, *args, **options):
        """ Handle the command call. """
        if options['list_categories']:
            categories = set(example.category for example in
                             ExampleObject.objects.all())
            for category in sorted(categories):
                print category
            return
        # We query the ids and work with them to avoid mongo timeout
        # Indeed, the download is very time consuming and can lead to the error
        #     Invalid cursor id <ID> on server
        if options['preload_all']:
            example_object_ids = [e.pk for e in ExampleObject.objects.all()]
        elif args:
            example_object_ids = [e.pk for e in ExampleObject.objects.filter(
                                  category__in=args)]
        else:
            raise "No Args"
        example_object = ExampleObject.objects.get(name='banana_1')
        preload_example_object(example_object, options)
        # for example_object_id in example_object_ids:
        #     example_object = ExampleObject.objects.get(id=example_object_id)
        #     preload_example_object(example_object, options)
