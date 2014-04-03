from django.db import models
   
from system_evaluation.models.evaluation_session import EvaluationSession

class IdentificationAttempt(models.Model):
    evaluation_session = models.ForeignKey(EvaluationSession)
    example = models.CharField(max_length=255)
    identification_succeed = models.BooleanField()
    identification_result = models.CharField(max_length=255)
    user_agreed = models.BooleanField()
    user_indentification = models.CharField(max_length=255)


