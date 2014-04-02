# -*- coding: utf-8 -*-

from random import randint
import tarfile, re, os, tempfile, re

img_folder_path = u"/home/julien/rgbd-dataset/images"
pcd_tars_path = u"/home/julien/rgbd-dataset/pcd_tars"
# pcd_tar_files = tarfile.open( u"/home/julien/dataset_pcd.tar" )

class ExampleManager:
    # returns the number of example available
    @staticmethod
    def size():
        return len(os.listdir( img_folder_path ))

    # retrieves one random pcd and img from the dataset
    # this function must be modified if the architecture of the dataset changes
    @staticmethod
    def get_random_model():
        index = randint(0, ExampleManager.size() - 1)
        img_path = os.listdir( img_folder_path )[index]
        return os.path.splitext( img_path )[0]

    @staticmethod
    def get_image(example_name):
        img_path = u"{}/{}.png".format(img_folder_path, example_name)
        return file(img_path)

    @staticmethod
    def get_pcd(example_name):
        object_name = re.search('[a-z_]+_[0-9]+', example_name).group(0)
        category_name = re.search('([a-z]+(_[a-z]+)*)+', example_name).group(0)
        tar_path = u"{}/{}.tar".format(
                pcd_tars_path,
                object_name
                )
        pcd_path = u"rgbd-dataset/{}/{}/{}.pcd".format(category_name, object_name, example_name)
        # the temporary file will be automatically deleted when pcd_file closes
        # pcd_file = tarfile.open( tar_path )
        archive = tarfile.open(tar_path)
        pcd_file = tempfile.NamedTemporaryFile()
        pcd_file.write( archive.extractfile( pcd_path ).read() )
        pcd_file.flush()
        pcd_file.seek(0)
        return pcd_file