from django.db import models

from djangotoolbox.fields import ListField
from django_mongodb_engine.fields import GridFSField
from gridfs import GridFS

from bs4 import BeautifulSoup as Soup
import urllib2

#from sketchup_models.testsoup import scrap_model, search_models

# see there : https://django-mongodb-engine.readthedocs.org/en/latest/tutorial.html

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


    def mesh(self):
        try:
            return self.mesh
        except:
            return self.super

    @staticmethod
    def find_google_id(google_id):
        try:
            return SketchupModel.objects.get(google_id=google_id)
        except SketchupModel.DoesNotExist:
            model = SketchupModel._scrap_model_page(google_id)
            model.save
            return model

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

        def absolute_path(relative_path):
            return "http://sketchup.google.com{0}".format(relative_path)

        model_url = absolute_path("/3dwarehouse/details?mid={}&prevstart=0").format(google_id)
        try:
            model = SketchupModel.objects.get(google_id=google_id)
        except SketchupModel.DoesNotExist:
            model = SketchupModel()
        model.google_id = google_id
        soup = Soup( urllib2.urlopen(model_url) )
        model.title = soup.select('#bylinetitle')[0].string
        model.text = soup.select('span#descriptionText')[0].string
        for tag in soup.select('a.fl'):
            if not model.tags.count(tag.string):
                model.tags.append(tag.string) 
        #if download_skp:
        #    model.skp = "okok"
        link_image = absolute_path(soup.select('#previewImage')[0]['src'] )
        model.image = urllib2.urlopen(link_image).read()
        model.save()
        return model

    @staticmethod
    def _scrap_search_engine(keywords):
        search_url = 'http://sketchup.google.com/3dwarehouse/search?q={0}&styp=m&scoring=t'.format(keywords)
        soup = Soup( urllib2.urlopen(search_url) )
        model_ids = []
        for div in soup.select('.searchresult'):
            model_ids.append(div['id'])
        return model_ids

