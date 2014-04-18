from django.db import models
from django_mongodb_engine.fields import GridFSField
from gridfs import GridFS, GridOut
import urllib2, tempfile, os, json, re

from sketchup_models.models import SketchupModel
from time import sleep

class WarehouseCache(models.Model):
    url = models.CharField(unique=True, max_length=500)
    ressource = GridFSField()

    @staticmethod
    def get_ressource(url):
        try:
            return WarehouseCache.objects.get(url=url).ressource.read()
        except WarehouseCache.DoesNotExist:
            print "Getting ressource {}".format( url )
            newEntry = WarehouseCache()
            newEntry.url = url
            newEntry.ressource = urllib2.urlopen(url).read()
            newEntry.save()
            return newEntry.ressource

class WarehouseScrapper():

    @staticmethod
    def search_for_models(keywords):
        models = []
        if keywords:
            model_ids = WarehouseScrapper._scrap_search_engine(keywords)
            for model_id in model_ids:
                models.append(SketchupModel.find_google_id(model_id))
                sleep(0.100)
        return models 

    @staticmethod
    def scrap_one_model(google_id):

        skp_binary_tag = ['s', 's13', 's8', 's7', 's6']
        img_binary_tag = ['lt', 'bot_st']

        def get_one_url(json_binary, tag_list):
            for tag in tag_list:
                if tag in json_binary:
                    print "tag {} found.".format(tag)
                    return json_binary[tag]['contentUrl']
            raise KeyError("Tags {} cannot be found.".format(tag_list))

        model_url = ("https://3dwarehouse.sketchup.com/3dw/GetEntity?id={}".
        format(google_id))
        model = SketchupModel()
        model.google_id = google_id
        json_data = json.loads(WarehouseCache.get_ressource( model_url) )     
        model.title = json_data['title']
        model.text = json_data['description']
        model.tags = json_data['tags']
        # retreive the image and skp url
        link_image = get_one_url(json_data['binaries'], img_binary_tag)
        link_skp = get_one_url(json_data['binaries'], skp_binary_tag)
        model.url_mesh = link_skp
        model.image = WarehouseCache.get_ressource(link_image)
        model.save()
        return model

    @staticmethod
    def _download_skp_and_convert_to_tri(model, url_skp):        
        # the mesh in store in temp and converted into a .tri file
        with tempfile.NamedTemporaryFile() as tmp_file:
            tmp_file.write( WarehouseCache.get_ressource(url_skp) )
            tmp_file.flush()
            cvt_cmd = 'WINEDEBUG=-all, ../bin/skp2tri.exe {0} {0}'
            if os.system(cvt_cmd.format(tmp_file.name)) != 0:
                raise ("Error on skp2tri conversion : model: {}, url: {}".
                    format(model.google_id, url) )
            tmp_file.seek(0)
            model.mesh = tmp_file.read()

    @staticmethod
    def _scrap_search_engine(keywords):
    	keywords = re.sub(' ', '%2B', keywords)
        search_url = (
            "https://3dwarehouse.sketchup.com/"
            "3dw/Search?startRow=1&endRow=16&calculateTotal=true"
            "&q&type=SKETCHUP_MODEL&class=entity&Lk=true&title={}"
            .format(keywords)
            )
        # print search_url
        json_data = json.loads( WarehouseCache.get_ressource( search_url ) )
        model_ids = []
        for entry in json_data['entries']:
            model_ids.append(entry['id'])
        return model_ids

