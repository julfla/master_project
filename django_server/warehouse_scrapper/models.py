""" Implementation of the SketchupWarehouse API. """

from django.db import models
from django_mongodb_engine.fields import GridFSField
import json
from urllib import urlencode
import urllib2

from sketchup_models.models import SketchupModel


SKETCHUP_API_URL = "https://3dwarehouse.sketchup.com/3dw"
IMG_TAGS = set(['lt', 'bot_st'])
SKP_TAGS = set(['s', 's13', 's8', 's7', 's6'])

NUMBER_OF_RESULTS = 32


class WarehouseCache(models.Model):

    """ A cache manager for http ressources. """

    url = models.CharField(unique=True, max_length=500)
    ressource = GridFSField()

    @staticmethod
    def get_ressource(url):
        """ Return url content after and cache in in db. """
        try:
            return WarehouseCache.objects.get(url=url).ressource.read()
        except WarehouseCache.DoesNotExist:
            print "Getting ressource {}".format(url)
            newEntry = WarehouseCache()
            newEntry.url = url
            newEntry.ressource = urllib2.urlopen(url).read()
            newEntry.save()
            return newEntry.ressource


def api_get(command, **params):
    """ Return a json object from the Sketchup API. """
    url = "{}/{}?{}".format(SKETCHUP_API_URL, command, urlencode(params))
    return json.loads(WarehouseCache.get_ressource(url))


def search_by_keywords(keywords, create_models=False):
    """ Search the API for the keywords, return a list of model_ids. """
    params = {'startRow': 1, 'endRow': NUMBER_OF_RESULTS, 'q': keywords,
              'type': 'SKETCHUP_MODEL', 'class': 'entity', 'Lk': True}
    json_data = api_get('Search', **params)
    if create_models:
        models = []
        for entry in json_data['entries']:
            try:
                model = SketchupModel()
                _parse_model_entry(model, entry)
                model.save()
                models.append(model)
            except KeyError as exception:
                print exception
        return models
    else:
        return [entry['id'] for entry in json_data['entries']]


def retreive_model(google_id):
    """ Return the SketchupModel corresponding to the google_id. """
    json_data = api_get("GetEntity", id=google_id)
    try:
        model = SketchupModel.objects.get(google_id=google_id)
    except SketchupModel.DoesNotExist:
        model = SketchupModel()
    try:
        _parse_model_entry(model, json_data)
        model.save()
        return model
    except KeyError:
        return None


def _parse_model_entry(model, json_data):
    model.google_id = json_data['id']
    model.title = json_data['title']
    model.text = json_data['description']
    # model.tags = json_data['tags']
    model.url_image = _extract_binary_url(
        model.google_id, json_data['binaryNames'], IMG_TAGS)
    model.url_mesh = _extract_binary_url(
        model.google_id, json_data['binaryNames'], SKP_TAGS)


def _extract_binary_url(google_id, binary_names, tags):
    """ Return the contentUrl for one of the tag if found in the json. """
    tags_found = list(set(tags) & set(binary_names))
    if tags_found:
        return "{}/{}?subjectClass=entity&subjectId={}&name={}".format(
            SKETCHUP_API_URL, "getbinary", google_id, tags_found[0])
    else:
        error_msg = "Any of {} tags not found in {} for model {}".format(
            tags, binary_names, google_id)
        raise KeyError(error_msg)


def _download_and_convert_skp2tri(url_skp):
    """ Return a file object of the skp ressource, converted to tri. """
    import os
    import tempfile
    temp_file = tempfile.NamedTemporaryFile()
    temp_file.write(WarehouseCache.get_ressource(url_skp))
    temp_file.flush()  # flush the file before the system call
    cvt_cmd = 'WINEDEBUG=-all, ../bin/skp2tri.exe {0} {0}'
    if os.system(cvt_cmd.format(temp_file.name)) != 0:
        raise ("Error on skp2tri conversion url: %s" % url_skp)
    else:
        temp_file.seek(0)
        return temp_file


class WarehouseScrapper():

    @staticmethod
    def search_for_models(keywords):
        models = []
        if keywords:
            model_ids = search_by_keywords(keywords)
            for model_id in model_ids:
                models.append(SketchupModel.find_google_id(model_id))
                # sleep(0.100)
        return models

    @staticmethod
    def scrap_one_model(google_id):
        return retreive_model(google_id)

    @staticmethod
    def _download_skp_and_convert_to_tri(model, url_skp):
        model.mesh = _download_and_convert_skp2tri(url_skp).read()
