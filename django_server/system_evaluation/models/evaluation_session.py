from django.db import models
from djangotoolbox.fields import ListField, EmbeddedModelField

from system_evaluation.models import ExampleObject, VideoSequence
from identifier.models import Identifier

from sklearn.neighbors import KNeighborsClassifier


class IdentificationAttempt(models.Model):

    """ One attempt of evaluation a VideoSequence and its result. """

    class Meta:
        app_label = 'system_evaluation'

    example_object_name = models.CharField(max_length=50)
    video_sequence_id = models.IntegerField()
    identification_succeed = models.BooleanField()
    identification_result = models.CharField(max_length=255)
    user_agreed = models.BooleanField(default=True)
    user_identification = models.CharField(max_length=255)
    new_category_learned = models.BooleanField(default=False)
    selected_model_ids = ListField()

    @property
    def video_sequence(self):
        example_object = ExampleObject.objects.get(
            name=self.example_object_name)
        return VideoSequence.objects.get(
            example_object=example_object, sequence_id=self.video_sequence_id)

    @video_sequence.setter
    def video_sequence(self, value):
        self.video_sequence_id = value.sequence_id
        self.example_object_name = value.example_object.name


class EvaluationSession(models.Model):

    """ Model corresponding to one session of user study evaluation. """

    class Meta:
        app_label = 'system_evaluation'

    attempts = ListField(EmbeddedModelField('IdentificationAttempt'))
    user = models.CharField(max_length=50, default="Anonymous")
    identifier = EmbeddedModelField()
    number_failures = models.IntegerField()
    agree_teaching = models.TextField()
    other_interface = models.TextField()
    action_robot = models.TextField()

    def __init__(self, *args, **kwargs):
        """ Overwrite to have a default classifier. """
        if 'identifier' not in kwargs:
            default = Identifier(classifier=KNeighborsClassifier())
            kwargs['identifier'] = default
        super(EvaluationSession, self).__init__(*args, **kwargs)
