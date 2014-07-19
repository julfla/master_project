from django.db import models

from djangotoolbox.fields import ListField
from django_mongodb_engine.fields import GridFSField
from gridfs import GridOut


class SketchupModel(models.Model):

    """ Model for a 3D model in SketchupWarehouse. """

    google_id = models.CharField(unique=True, max_length=255)
    title = models.CharField(max_length=255)
    text = models.TextField()
    tags = ListField()
    image = GridFSField()
    url_mesh = models.TextField()
    _mesh = GridFSField()
    # similat_objects

    @property
    def mesh(self):
        """ Return the content of the mesh file, download it if needed. """
        if isinstance(self._mesh, GridOut):
            self._mesh.seek(0)
            return self._mesh.read()
        elif self.url_mesh and not self._mesh:
            from warehouse_scrapper.models import _download_and_convert_skp2tri
            mesh_file = _download_and_convert_skp2tri(self.url_mesh)
            self._mesh = mesh_file.read()
            print mesh_file.read()
            self.save()
        return self._mesh

    @mesh.setter
    def mesh(self, value):
        """ Setter for the property. """
        self._mesh = value

    def __str__(self):
        """ Overwrite. """
        return self.google_id

    @staticmethod
    def find_google_id(google_id):
        """ Return the model matching the google_id, dowload it if needed. """
        try:
            return SketchupModel.objects.get(google_id=google_id)
        except SketchupModel.DoesNotExist:
            from warehouse_scrapper.models import retreive_model
            return retreive_model(google_id)
