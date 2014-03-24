from django.http import HttpResponse, HttpResponseNotFound
import datetime

from django.shortcuts import render_to_response
from django.template import RequestContext

from sketchup_models.models import SketchupModel

def search_models(request, keywords):
    models = SketchupModel.search_warehouse(keywords)   
    return render_to_response('default.html', {'models': models}, 
        context_instance=RequestContext(request))

def model_image(request, google_id):
    try:
        model = SketchupModel.find_google_id(google_id)
        return HttpResponse(model.image.read(), mimetype="image/png")
    except SketchupModel.DoesNotExist:
        return HttpResponseNotFound()

def model_mesh(request, google_id):
    try:
        model = SketchupModel.find_google_id(google_id)
        if model.mesh == None:
            return HttpResponseNotFound()
        response = HttpResponse(model.mesh, mimetype="text/plain")
        response['Content-Disposition'] = 'attachment; filename= %s.tri' % model
        return response
    except SketchupModel.DoesNotExist, Sketchup.AttributeError:
        return HttpResponseNotFound()

def scrap_model(request, google_id):
    scrap.scrap_model(google_id)
    return search_models(request, "")


