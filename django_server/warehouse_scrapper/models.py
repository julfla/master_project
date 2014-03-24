from gridfs import GridFS, GridOut
from bs4 import BeautifulSoup as Soup
import urllib2, tempfile, os, json, re

from sketchup_models.models import SketchupModel

class WarehouseScrapper():

    @staticmethod
    def search_for_models(keywords):
        models = []
        if keywords:
            model_ids = WarehouseScrapper._scrap_search_engine(keywords)
            for model_id in model_ids:
                models.append(SketchupModel.find_google_id(model_id))
        return models 

    @staticmethod
    def scrap_one_model(google_id):
        model_url = ("https://3dwarehouse.sketchup.com/3dw/GetEntity?id={}".
        format(google_id))
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
            if 'types' in binary and binary['types'] == 'SKP':
                model.url_ink_skp = binary['url']
                break
        model.image = urllib2.urlopen(link_image).read()
        model.save()
        return model

    @staticmethod
    def _download_skp_and_convert_to_tri(model, url_skp):        
        # the mesh in store in temp and converted into a .tri file
        with tempfile.NamedTemporaryFile() as tmp_file:
            tmp_file.write( urllib2.urlopen(url_skp).read() )
            tmp_file.flush()
            cvt_cmd = 'WINEDEBUG=-all, ../bin/skp2tri.exe {0} {0}'
            os.system(cvt_cmd.format(tmp_file.name) )
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
        print search_url
        json_data = json.load( urllib2.urlopen( search_url ) )
        model_ids = []
        for entry in json_data['entries']:
            model_ids.append(entry['id'])
        return model_ids

