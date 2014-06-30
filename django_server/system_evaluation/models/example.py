from django.db import models
from django_mongodb_engine.fields import GridFSField
from gridfs import GridOut

from zlib import compress, decompress
import re, tempfile


def get_example_path(instance, filename):
    """ Return the relative path where to store a example file.

    File will be stored in :
        MEDIA_ROOT/rgbd-dataset/<category>/<object_name>
    """
    from os.path import basename
    return "{}/{}/{}".format(instance.category,
                             instance.object_name,
                             basename(filename))


class Example(models.Model):

    class Meta:
        app_label = 'system_evaluation'

    name = models.CharField(unique=True, max_length=255)
    pcd = models.FileField(upload_to=get_example_path)
    image = models.FileField(upload_to=get_example_path)

    @property
    def category(self):
        return re.search('([a-z]+(_[a-z]+)*)+', self.name).group(0)

    @property
    def object_name(self):
        return re.search('[a-z_]+_[0-9]+', self.name).group(0)

    @staticmethod
    def filter_categories(list_categories):
        regex = '({})(_[0-9]+)+'.format(
            '|'.join(list_categories)
            )
        return Example.objects.filter(name__regex=regex)

    @staticmethod
    def get_random(list_categories):
        from random import randint
        examples = Example.filter_categories(list_categories)
        index = randint(0, len(examples) - 1)
        return examples[index]

# Receive the pre_delete signal and delete the file associated with the example
from django.db.models.signals import pre_delete
from django.dispatch.dispatcher import receiver


@receiver(pre_delete, sender=Example)
def mymodel_delete(sender, instance, **kwargs):
    # Pass false so FileField doesn't save the model.
    instance.image.delete(False)
    instance.pcd.delete(False)
