from django.db import models
from pointcloud.models import PointCloud


def get_example_path(instance, filename):
    """ Return the relative path where to store a example file.

    File will be stored in :
        MEDIA_ROOT/rgbd-dataset/<category>/<object_name>
    """
    from os.path import basename
    return "{}/{}/{}".format(instance.category,
                             instance.object_name,
                             basename(filename))


class ExampleObject(models.Model):

    """ An object from the dataset with several video sequences.

    The model contains also information regarding how to download the dataset
    and where is it store is downloaded.
    """

    name = models.CharField(max_length=50, unique=True)
    category = models.CharField(max_length=50)
    url_pcd_tar = models.URLField()
    url_image_tar = models.URLField()
    pcd_tar = models.FileField(upload_to="rgbd-dataset/pcd_tar")
    image_tar = models.FileField(upload_to="rgbd-dataset/image_tar")

    class Meta:
        app_label = "system_evaluation"


class VideoSequence(models.Model):

    """ Sequence of frames from the dataset. """

    sequence_id = models.IntegerField()
    example_object = models.ForeignKey(ExampleObject,
                                       related_name="sequences")

    class Meta:
        unique_together = (("sequence_id", "example_object"),)
        app_label = "system_evaluation"


class Frame(models.Model):

    """ One frame of an example sequence.

    It contains the reference on how to retreive the pcd and image.
    """

    frame_id = models.IntegerField()
    video_sequence = models.ForeignKey(VideoSequence,
                                       related_name="frames")
    pcd_member_name = models.CharField(max_length=255)
    image_member_name = models.CharField(max_length=255)
    # _distribution = embededfield blabla

    class Meta:
        unique_together = (("frame_id", "video_sequence"),)
        app_label = 'system_evaluation'

    @property
    def pointcloud(self):
        """ Extract the pointcloud from the ExampleObject archive. """
        import tarfile
        from tempfile import NamedTemporaryFile
        from django_server.settings import MEDIA_ROOT
        example_object = self.video_sequence.example_object
        temp_file = NamedTemporaryFile(delete=True)
        pcd_tar = tarfile.open(MEDIA_ROOT + example_object.pcd_tar.name)
        temp_file.write(pcd_tar.extractfile(self.pcd_member_name).read())
        temp_file.flush()
        pointcloud = PointCloud.load_pcd(temp_file.name)
        return pointcloud


# Deletion of files when deleting a ExampleObject
from django.db.models.signals import pre_delete
from django.dispatch.dispatcher import receiver


@receiver(pre_delete, sender=ExampleObject)
def mymodel_delete(sender, instance, **kwargs):
    # Pass false so FileField doesn't save the model.
    instance.pcd_tar.delete(False)
    instance.image_tar.delete(False)
