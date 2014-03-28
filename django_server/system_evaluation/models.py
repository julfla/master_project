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
        res['pcd'] = pcd_path
        res['img'] = img_path
        return res

    def get_image(self, path):
        return img_tar_file.extractfile(path)

    def get_pcd(self, path):
        return pcd_tar_file.extractfile(path)



class SystemEvaluation(models.Model):
    user = models.CharField(max_length = 50, default="Anonymous")
    
class IdentificationAttempt(models.Model):
    evaluation = models.ForeignKey(SystemEvaluation)
    img_path = models.CharField(max_length=255)
    pcd_path = models.CharField(max_length=255)
    identification_result = models.CharField(max_length=255)
    user_agreed = models.BooleanField()
    user_indentification = models.CharField(max_length=255)


