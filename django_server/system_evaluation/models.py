# -*- coding: utf-8 -*-

from django.db import models
from random import randint
import tarfile, re

img_tar_file = tarfile.open( u"/home/julien/dataset_img.tar" )
pcd_tar_file = tarfile.open( u"/home/julien/dataset_pcd.tar" )

class ExampleManager:
    # returns the number of example available
    def size(self):
        return len(pcd_tar_file.getmembers())

    # retrieves one random pcd and img from the dataset
    # this function must be modified if the architecture of the dataset changes
    def get_random_model(self):
        index = randint(0, self.size() - 1)
        pcd_path = pcd_tar_file.getnames()[index]
        img_path = re.sub('.pcd', '.png', pcd_path)

        res = {}
        res['pcd'] = pcd_tar_file.extractfile(pcd_path)
        res['img'] = img_tar_file.extractfile(img_path)
        return res

class SystemEvaluation(models.Model):
    pass

