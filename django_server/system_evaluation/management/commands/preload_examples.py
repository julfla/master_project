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
    import shutil
    import os.path
    sequences = list(example_object.sequences.all())
    frames = Frame.objects.filter(video_sequence__in=sequences)
    if not options['force']:
        frames = frames.filter(_pointcloud__isnull=True)
    if not frames.exists():
        return  # Nothing to process
    if options['download_folder'] is None:
        archive_dir = dowload_archives([example_object])
    else:
        archive_dir = options['download_folder']
    pcd_tar_path = "{}/pcd_tar/{}.tar".format(archive_dir,
                                                  example_object.name)
    pcd_tar = tarfile.open(pcd_tar_path)
    tmp_dir = tempfile.mkdtemp()
    pcd_tar.extractall(tmp_dir)
    # pcd_members = dict((m.name, m) for m in pcd_tar.getmembers())
    for frame in frames.iterator():
        current_pcd = os.path.join(tmp_dir, frame.pcd_member_name())
        pointcloud = PointCloud.load_pcd(current_pcd)
        frame._pointcloud = pointcloud
        frame.save()
    shutil.rmtree(tmp_dir)
    if options['download_folder'] is None:
        shutil.rmtree(archive_dir)


def save_distributions(example_object, options):
    """ Store distributions into the db. """
    # TODO will be extremly slow if the pointcloud are not stored.
    sequences = list(example_object.sequences.all())
    frames = Frame.objects.filter(video_sequence__in=sequences)
    if not options['force']:
        frames = frames.filter(_distribution__isnull=True)
    if not frames.exists() or not options['save_distribution']:
        return  # No process needed
    for frame in frames.iterator():
        frame.get_distribution(True, options['force'])


def dowload_archives(example_objects, download_folder=None):
    """ For the list of ExampleObject, dowload the archives.

    If download_folder is None, if will be dowloaded to a tmp directory.
    The function returns the path to the dowload directory.
    """
    if download_folder is None:
        import tempfile
        download_folder = tempfile.mkdtemp()
    import os
    import os.path
    print "Dowloading archives to %s" % download_folder
    # print "Dowloading image archives"
    # list_urls = [e.url_image_tar for e in example_objects]
    # os.system("wget -P {} -nc {}".format(
    #     os.path.join(download_folder, "image_tar"),
    #     " ".join(list_urls)))
    print "Dowloading pcd archives"
    list_urls = [e.url_pcd_tar for e in example_objects]
    os.system("wget -P {} -nc {}".format(
        os.path.join(download_folder, "pcd_tar"),
        " ".join(list_urls)))
    return download_folder


def save_images(example_object, download_folder=None):
    """ Save the images of the frames, dowload if necessary."""
    # TODO
    pass


class Command(BaseCommand):

    """ Django Command to dowload the dataset of example objects. """

    help = ('Dowload some examples from an online dataset\n'
            'Usage is : \n'
            '    preload_models [options] CATEGORY1 CATEGORY2 ...\n'
            '    preload_models [options] --all')

    option_list = BaseCommand.option_list + (
        make_option('-l', '--list',
                    dest='list_categories',
                    action='store_true',
                    help='List the available categories.'),
        make_option('-a', '--all',
                    dest='preload_all',
                    action='store_true',
                    help='Process all the dataset.'),
        make_option('-D', '--download_folder',
                    dest='download_folder',
                    help='Store/load the dataset archives in this directory.'),
        make_option('-p', '--save_pointcloud',
                    dest='save_pointcloud',
                    action='store_true',
                    help='Save the PointClouds in db.'),
        make_option('-d', '--save_distribution',
                    dest='save_distribution',
                    action='store_true',
                    help='Save the ShapeDistributions in db.'),
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
        if options['preload_all']:
            example_objects = ExampleObject.objects
        elif args:
            example_objects = ExampleObject.objects.filter(category__in=args)
        else:
            print self.help
            raise ValueError

        if not example_objects.exists():
            print "No examples could be found"
            print ("Did you load the fixture " +
                   "system_evaluation/fixtures/initial_data.json ?")
            return

        # We donwload all the corresponding archives, without overwriting
        if options['download_folder']:
            dowload_archives(example_objects.all(), options['download_folder'])

        if not (options['force'] or options['save_pointcloud'] or
           options['save_distribution']):
            return 0  # Nothing to process

        # We query the ids and work with them to avoid mongo timeout
        # Indeed, the download is very time consuming and can lead to the error
        #     Invalid cursor id <ID> on server
        number_of_objects = example_objects.count()
        for index, example_object_id in enumerate(
           [e.pk for e in example_objects.all()]):
            example_object = ExampleObject.objects.get(id=example_object_id)
            print "Object {} ({}/{})".format(example_object.name,
                index, number_of_objects)
            if options['save_pointcloud']:
                save_pointclouds(example_object, options)
                print "    PointClouds processed"
            if options['save_distribution']:
                save_distributions(example_object, options)
                print "    Distributions processed"
