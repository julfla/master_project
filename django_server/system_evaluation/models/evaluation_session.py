from django.db import models
from djangotoolbox.fields import ListField, EmbeddedModelField
from identifier.models import Identifier

class IdentificationAttempt(models.Model):
    example = models.CharField(max_length=255)
    identification_succeed = models.BooleanField()
    identification_result = models.CharField(max_length=255)
    user_agreed = models.BooleanField()
    user_indentification = models.CharField(max_length=255)

class EvaluationSession(models.Model):
    attempts = ListField(EmbeddedModelField('IdentificationAttempt'))
    user = models.CharField(max_length = 50, default="Anonymous")
    identifier = models.ForeignKey('identifier.Identifier')

    def save(self, *args, **kwargs):
    	if self.identifier_id is None:
    		identifier = Identifier()
    		identifier.save()
    		self.identifier = identifier
        super(EvaluationSession, self).save(*args, **kwargs)
