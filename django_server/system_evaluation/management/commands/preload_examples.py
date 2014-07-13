""" Define a Django Command to seed the examples database. """

from django.core.management.base import BaseCommand
from django.core.files import File
from optparse import make_option
from system_evaluation.models import ExampleObject, VideoSequence, Frame
from pointcloud.models import PointCloud
from shape_distribution.models import ShapeDistribution
from django_server.settings import MEDIA_ROOT

import tarfile
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


def save_pointclouds(example_object, options):
    """ Extract pointclouds from tar archive. """
    sequences = list(example_object.sequences.all())
    frames = Frame.objects.filter(video_sequence__in=sequences)
    if not options['force']:
        frames = frames.filter(_pointcloud__isnull=True)
    if not frames.exists():
        return  # Nothing to process
    temp_dir = tempfile.mkdtemp()
    pcd_tar = tarfile.open(MEDIA_ROOT + example_object.pcd_tar.name)
    pcd_tar.extractall(temp_dir)  # Extract all pcds
    # We add the temp_dir to options for further process
    options['pcds_temp_dir'] = temp_dir
    if not options['save_pointcloud']:
        # We extracted the archive in order to enfaster distribution process
        return  # But we don't need to process the pointclouds
    for frame in frames.iterator():
        pcd_path = "{}/{}".format(temp_dir, frame.pcd_member_name())
        pointcloud = PointCloud.load_pcd(pcd_path)
        frame._pointcloud = pointcloud
        frame.save()


def save_distributions(example_object, options):
    """ Store pointclouds and/or distributions into the db. """
    sequences = list(example_object.sequences.all())
    frames = Frame.objects.filter(video_sequence__in=sequences)
    if not options['force']:
        frames = frames.filter(_distribution__isnull=True)
    if not frames.exists() or not options['save_distribution']:
        return  # No process needed
    temp_dir = options['pcds_temp_dir']
    for frame in frames.iterator():
        if frame._pointcloud:
            pointcloud = frame._pointcloud
        else:
            pcd_path = "{}/{}".format(temp_dir, frame.pcd_member_name())
            pointcloud = PointCloud.load_pcd(pcd_path)
        frame._distribution = ShapeDistribution.compute(pointcloud)
        frame.save()


def preload_example_object(example_object, options):
    """ Dowload the dataset archives and create the sequences and frames. """
    # Dowload the pcd archive file if needed
    print "Processing example %s" % example_object.name
    if not example_object.pcd_tar:
        temp_file = dowload_with_progress_bar(example_object.url_pcd_tar)
        file_name = "%s_pcd.tar" % example_object.name
        example_object.pcd_tar.save(file_name, File(temp_file))
    # Dowload the image archive file if needed
    if not example_object.image_tar:
        temp_file = dowload_with_progress_bar(example_object.url_image_tar)
        file_name = "%s_image.tar" % example_object.name
        example_object.image_tar.save(file_name, File(temp_file))
    pcd_tar = tarfile.open(MEDIA_ROOT + example_object.pcd_tar.name)
    # We iterate over all example's frames
    for pcd_member_name in pcd_tar.getnames():
        # The path end with <...>_<sequence_id>_<frame_id>.pcd
        frame_number = int(pcd_member_name.split('_')[-1].split('.')[0])
        sequence_number = int(pcd_member_name.split('_')[-2])
        # We keep only the one fith of the frames, starting from the 1st
        if frame_number % 5 == 1:
            sequence, _ = VideoSequence.objects.get_or_create(
                example_object=example_object, sequence_id=sequence_number)
            frame, _ = Frame.objects.get_or_create(
                video_sequence=sequence, frame_id=frame_number)
    number_frames = 0
    for sequence in example_object.sequences.iterator():
        number_frames += sequence.frames.count()
    print "{} : {} sequences and {} frames.".format(
        example_object.name, example_object.sequences.count(), number_frames)
    return example_object


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
        make_option('-f', '--force',
                    dest='force',
                    action='store_true',
                    help='Overwrite values of pointcloud or distribution.'),
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
        for example_object_id in example_object_ids:
            example_object = ExampleObject.objects.get(id=example_object_id)
            example_object = preload_example_object(example_object, options)
            if (options['force'] or options['save_pointcloud'] or
               options['save_distribution']):
                save_pointclouds(example_object, options)
                save_distributions(example_object, options)
            if 'pcd_temp_dir' in options:
                import shutil
                shutil.rmtree(options['pcd_temp_dir'])
