from django.db import models
from django_mongodb_engine.fields import GridFSField
from gridfs import GridOut

from zlib import compress, decompress
import re, tempfile

class Example(models.Model):
    name = models.CharField(unique=True, max_length=255)
    _compressed = models.BooleanField(default=False)
    _pcd_raw = GridFSField()
    _image_raw = GridFSField()

    def compress_gridfsfield(self, data):
        import zlib
        if 'read' in dir(data): data = data.read() 
        if self.compressed:
            return zlib.compress( data )
        else:
            return data

    def decompress_gridfsfield(self, data):
        import zlib
        if isinstance( data, GridOut):
            data.seek(0)
            data = data.read()
        if self.compressed:
            return zlib.decompress( data )
        else:
            return data

    @property
    def compressed(self):
        return self._compressed

    @compressed.setter
    def compressed(self, value):
        if value ^ self.compressed:
            pcd_data = self.pcd_file
            image_data = self.image_file
            _compressed = value
            self.pcd_file = pcd_data
            self.image_file = image_data

    @property
    def pcd_file(self):
        tmpfile = tempfile.NamedTemporaryFile()
        tmpfile.write( self.decompress_gridfsfield(self._pcd_raw) )
        tmpfile.flush()        
        tmpfile.seek(0)
        return tmpfile

    @pcd_file.setter
    def pcd_file(self, value):
        self._pcd_raw = self.compress_gridfsfield(value)
    
    @property
    def image_file(self):
        tmpfile = tempfile.NamedTemporaryFile()
        tmpfile.write( self.decompress_gridfsfield(self._image_raw) )
        tmpfile.flush()
        tmpfile.seek(0)
        return tmpfile

    @image_file.setter
    def image_file(self, value):
        self._image_raw = self.compress_gridfsfield(value)

    @property
    def category(self):
        return re.search('([a-z]+(_[a-z]+)*)+', self.name).group(0)

    @property
    def object_name(self):
        return re.search('[a-z_]+_[0-9]+', self.name).group(0)

    @staticmethod
    def filter_categories(list_categories):
        temp = []
        for example in Example.objects.all():
            if example.category in list_categories:
                temp.append(example)
        return temp

    @staticmethod
    def get_random(list_categories):
        from random import randint
        examples = Example.filter_categories(list_categories)
        index = randint(0, len(examples) - 1)
        return examples[index]

