from django.db import models

class EvaluationSession(models.Model):
    user = models.CharField(max_length = 50, default="Anonymous")

    def new_attempt(self):
    	from system_evaluation.models import IdentificationAttempt
    	attempt = IdentificationAttempt()
    	attempt.evaluation_session = self
    	return attempt
