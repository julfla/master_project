from django.db import models

from djangotoolbox.fields import ListField
from django_mongodb_engine.fields import GridFSField
from gridfs import GridOut

import warehouse_scrapper.models

class CategoryField(ListField):
    def formfield(self, **kwargs):
        return models.Field.formfield(self, StringListField, **kwargs)

class SketchupModel(models.Model):
    google_id = models.CharField(unique=True, max_length=255)
    title = models.CharField(max_length=255)
    text = models.TextField()
    tags = CategoryField()
    image = GridFSField()
    _mesh = GridFSField()
    # similat_objects

    @property
    def mesh(self):
        if isinstance( self._mesh, GridOut):
            return self._mesh.read()
        else:
            return self._mesh

    @mesh.setter
    def mesh(self, value):
        self._mesh = value

    def __str__(self):
        return self.google_id

    @staticmethod
    def find_google_id(google_id):
        try:
            return SketchupModel.objects.get(google_id=google_id)
        except SketchupModel.DoesNotExist:
            WarehouseScrapper.scrap_one_model(google_id)
            return SketchupModel.objects.get(google_id=google_id)

