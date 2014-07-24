""" Implementation of the SketchupWarehouse API. """

from django.db import models
from django_mongodb_engine.fields import GridFSField
import json
from urllib import urlencode
import urllib2

from sketchup_models.models import SketchupModel


SKETCHUP_API_URL = "https://3dwarehouse.sketchup.com/3dw"
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


def search_by_keywords(keywords):
    """ Search the API for the keywords, return a list of model_ids. """
    params = {'startRow': 1, 'endRow': NUMBER_OF_RESULTS, 'q': keywords,
              'type': 'SKETCHUP_MODEL', 'class': 'entity', 'Lk': True}
    json_data = api_get('Search', **params)
    return [entry['id'] for entry in json_data['entries']]


def retreive_model(google_id):
    """ Return the SketchupModel corresponding to the google_id. """
    json_data = api_get("GetEntity", id=google_id)
    try:
        model = SketchupModel.objects.get(google_id=google_id)
    except SketchupModel.DoesNotExist:
        model = SketchupModel(google_id=google_id)
    try:
        model.title = json_data['title']
        model.text = json_data['description']
        model.tags = json_data['tags']
        url_image = _parse_image_url(json_data['binaries'])
        model.url_mesh = _parse_skp_url(json_data['binaries'])
        model.image = WarehouseCache.get_ressource(url_image)
        model.save()
    except KeyError:
        return None
    return model


def _parse_one_of_contentUrl(json_binaries, tag_set):
    """ Return the contentUrl for one of the tag if found in the json. """
    tags_found = list(tag_set & set(json_binaries.keys()))
    if tags_found:
        return json_binaries[tags_found[0]]['contentUrl']
    else:
        error_msg = "Any of {} tags not found in {}".format(
            tag_set, json_binaries)
        raise KeyError(error_msg)


def _parse_image_url(json_binaries):
    """ Return the image url from the json binaries list. """
    img_tags = set(['lt', 'bot_st'])
    return _parse_one_of_contentUrl(json_binaries, img_tags)


def _parse_skp_url(json_binaries):
    """ Return the skp url from the json binaries list. """
    skp_tags = set(['s', 's13', 's8', 's7', 's6'])
    return _parse_one_of_contentUrl(json_binaries, skp_tags)


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
