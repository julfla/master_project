from django.db import models
from djangotoolbox.fields import EmbeddedModelField
from pointcloud.models import PointCloud
from shape_distribution.models import ShapeDistribution


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
    _distribution = EmbeddedModelField(ShapeDistribution, null=True)
    _pointcloud = EmbeddedModelField(PointCloud, null=True)

    class Meta:
        unique_together = (("frame_id", "video_sequence"),)
        app_label = 'system_evaluation'

    def _member_name(self):
        """ Common part between pcd_member_name and image_member_name. """
        object_name = self.video_sequence.example_object.name
        category = self.video_sequence.example_object.category
        frame_name = "{}_{}_{}".format(
            object_name, self.video_sequence.sequence_id, self.frame_id)
        return "rgbd-dataset/{}/{}/{}".format(
            category, object_name, frame_name)

    def pcd_member_name(self):
        """ Return the path of the pcd inside the tar archive. """
        return self._member_name() + ".pcd"

    def image_member_name(self):
        """ Return the path of the image inside the tar archive. """
        return self._member_name() + "_crop.png"

    @property
    def distribution(self, save=True, force_computation=False):
        """ Distribution of the frame, computed if needed, can be slow. """
        if not self._distribution or force_computation:
            distribution = ShapeDistribution.compute(self.pointcloud)
            if save:
                self._distribution = distribution
                self.save()
        else:
            distribution = self._distribution
        return distribution

    @property
    def pointcloud(self, save=False):
        """ PointCloud of the frame, very slow if read from the archive. """
        if not self._pointcloud:
            pointcloud = self.compute_pointcloud()
            if save:
                self._pointcloud = pointcloud
                self.save()
        else:
            pointcloud = self._pointcloud
        return pointcloud

    def compute_pointcloud(self):
        """ Extract the pointcloud from the ExampleObject archive. """
        import tarfile
        from tempfile import NamedTemporaryFile
        from django_server.settings import MEDIA_ROOT
        example_object = self.video_sequence.example_object
        temp_file = NamedTemporaryFile(delete=True)
        pcd_tar = tarfile.open(MEDIA_ROOT + example_object.pcd_tar.name)
        temp_file.write(pcd_tar.extractfile(self.pcd_member_name()).read())
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
