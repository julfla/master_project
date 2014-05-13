# -*- coding: utf-8 -*-

from random import randint
import tarfile, re, os, tempfile, re

img_folder_path = u"/home/julien/rgbd-dataset/images"
pcd_tars_path = u"/home/julien/rgbd-dataset/pcd_tars"
# pcd_tar_files = tarfile.open( u"/home/julien/dataset_pcd.tar" )

class ExampleManager:
    # returns the number of example available
    @staticmethod
    def list_examples(list_categories=None):
        examles = os.listdir( img_folder_path )
        if list_categories is None:
            return examles
        exp = "\A({})[0-9_]+".format( '|'.join(list_categories) )
        m = re.compile(exp)
        res = [f for f in examles if m.search(f)]
        return map( lambda x: os.path.splitext(x)[0], res)

    # retrieves one random pcd and img from the dataset
    # this function must be modified if the architecture of the dataset changes
    @staticmethod
    def get_random_example(list_categories=['banana','bowl']):
        number_examples = len(ExampleManager.list_examples(list_categories))
        index = randint(0, number_examples - 1)
        return ExampleManager.list_examples(list_categories)[index]

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
        archive = tarfile.open(tar_path)
        pcd_file = tempfile.NamedTemporaryFile()
        pcd_file.write( archive.extractfile( pcd_path ).read() )
        pcd_file.flush()
        pcd_file.seek(0)
        return pcd_file