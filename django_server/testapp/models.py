from django.db import models

from djangotoolbox.fields import ListField
from django_mongodb_engine.fields import GridFSField
from gridfs import GridFS

# see there : https://django-mongodb-engine.readthedocs.org/en/latest/tutorial.html

from .forms import StringListField

class CategoryField(ListField):
    def formfield(self, **kwargs):
        return models.Field.formfield(self, StringListField, **kwargs)

class SketchupModel(models.Model):
    google_id = models.CharField(max_length=255)
    title = models.CharField(max_length=255)
    text = models.TextField()
    tags = CategoryField()
    image = GridFSField()
    mesh = GridFSField()
    # similat_objects

    def __str__(self):
        return self.google_id


