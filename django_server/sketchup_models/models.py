from django.db import models

from djangotoolbox.fields import ListField
from django_mongodb_engine.fields import GridFSField
from gridfs import GridFS

from bs4 import BeautifulSoup as Soup
import urllib2, tempfile, os, httplib, json

class CategoryField(ListField):
    def formfield(self, **kwargs):
        return models.Field.formfield(self, StringListField, **kwargs)

class SketchupModel(models.Model):
    google_id = models.CharField(unique=True, max_length=255)
    title = models.CharField(max_length=255)
    text = models.TextField()
    tags = CategoryField()
    image = GridFSField()
    mesh = GridFSField()
    # similat_objects

    def __str__(self):
        return self.google_id    

    @staticmethod
    def find_google_id(google_id):
        try:
            return SketchupModel.objects.get(google_id=google_id)
        except SketchupModel.DoesNotExist:
            SketchupModel._scrap_model_page(google_id)
            return SketchupModel.objects.get(google_id=google_id)

    @staticmethod
    def search_warehouse(keywords):
        models = []
        if keywords:
            model_ids = SketchupModel._scrap_search_engine(keywords)
            for model_id in model_ids:
                models.append(SketchupModel.find_google_id(model_id))
        return models  

    @staticmethod
    def _scrap_model_page(google_id):

        model_url = ("https://3dwarehouse.sketchup.com/3dw/GetEntity?id={}".
        format(google_id))
        try:
            model = SketchupModel.objects.get(google_id=google_id)
        except SketchupModel.DoesNotExist:
            model = SketchupModel()
        model.google_id = google_id
        json_data = json.load(urllib2.urlopen(model_url))     
        model.title = json_data['title']
        model.text = json_data['description']
        model.tags = json_data['tags']
        # retreive the image
        link_image = json_data['binaries']['lt']['url']
        link_skp = None # the model can have no skp file associated with
        binary_names = json_data['binaryNames']
        for binary_name in binary_names:
            binary = json_data['binaries'][binary_name]
            if binary['types'] == 'SKP':
                link_skp = binary['url']             
        if not link_skp == None:
            # the mesh in store in temp and converted into a .tri file
            with tempfile.NamedTemporaryFile() as tmp_file:
                tmp_file.write( urllib2.urlopen(link_skp).read() )
                tmp_file.flush()
                cvt_cmd = 'WINEDEBUG=-all, ../bin/skp2tri.exe {0} {0}'
                os.system(cvt_cmd.format(tmp_file.name) )
                tmp_file.seek(0)
                model.mesh = tmp_file.read()
        model.image = urllib2.urlopen(link_image).read()
        model.save()
        return model

    @staticmethod
    def _scrap_search_engine(keywords):
        search_url = (
           #q&type=SKETCHUP_MODEL&source&title=fork&description&sortBy=title%20ASC&createUserDisplayName&createUserId&modifyUserDisplayName&class=entity&Lk=true
            "https://3dwarehouse.sketchup.com/"
            "3dw/Search?startRow=1&endRow=16&calculateTotal=true"
            "&q&type=SKETCHUP_MODEL&class=entity&Lk=true&title={}".format(keywords)
            )
        json_data = json.load( urllib2.urlopen( search_url ) )
        model_ids = []
        for entry in json_data['entries']:
            model_ids.append(entry['id'])
        return model_ids

