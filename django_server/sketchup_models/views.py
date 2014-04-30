from django.http import HttpResponse, HttpResponseNotFound
import datetime

from django.shortcuts import render_to_response, get_object_or_404
from django.template import RequestContext

from sketchup_models.models import SketchupModel
from warehouse_scrapper.models import WarehouseScrapper

def index(request):
    keywords = request.GET.get('keywords')
    print keywords
    if keywords == None:
        models = []
    else:
        models = WarehouseScrapper.search_for_models(keywords)
    return render_to_response('default.html', {'models': models}, 
        context_instance=RequestContext(request))

def detailed_view(request, google_id):
    model = SketchupModel.find_google_id(google_id)
    print "model {} has {} views.".format(model, model.partialview_set.count())
    if model is None:
        return HttpResponseNotFound()
    return render_to_response('model_detailed_view.html', {'model':model},
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


