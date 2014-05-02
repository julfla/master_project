from django.db import models
from djangotoolbox.fields import EmbeddedModelField
from identifier.models import Identifier

class EvaluationSession(models.Model):
    user = models.CharField(max_length = 50, default="Anonymous")
    identifier = models.ForeignKey('identifier.Identifier')

    def new_attempt(self):
    	from system_evaluation.models import IdentificationAttempt
    	attempt = IdentificationAttempt()
    	attempt.evaluation_session = self
    	return attempt

    def save(self, *args, **kwargs):
    	if self.identifier_id is None:
    		identifier = Identifier()
    		identifier.save()
    		self.identifier = identifier
        super(EvaluationSession, self).save(*args, **kwargs)
