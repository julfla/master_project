from django.db import models

from djangotoolbox.fields import ListField

# see there : https://django-mongodb-engine.readthedocs.org/en/latest/tutorial.html

class SketchupModel(models.Model):
    google_id = models.CharField()
    title = models.CharField()
    text = models.TextField()
    tags = ListField()
    # image = GridFSField()
    # similat_objects

    def _str_(self):
        google_id
