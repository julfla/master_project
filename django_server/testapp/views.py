from django.http import HttpResponse, HttpResponseNotFound
import datetime

from django.shortcuts import render_to_response
from django.template import RequestContext

from testapp.models import SketchupModel

def search_models(request, keywords):
    models = SketchupModel.search_warehouse(keywords)   
    return render_to_response('default.html', {'time': datetime.datetime.now(), 'models': models}, 
        context_instance=RequestContext(request))

def model_image(request, google_id):
    try:
        model = SketchupModel.objects.get(google_id=google_id)
        return HttpResponse(model.image.read(), mimetype="image/png")
    except SketchupModel.DoesNotExist:
        return HttpResponseNotFound()

def model_mesh(request, google_id):
    try:
        model = SketchupModel.objects.get(google_id=google_id)
        response = HttpResponse(model.mesh.read(), mimetype="application/octet-stream")
        response['Content-Disposition'] = 'attachment; filename= %s.skp' % model
        return response
    except SketchupModel.DoesNotExist:
        return HttpResponseNotFound("<h1>Page not found</h1>")

def scrap_model(request, google_id):
    scrap.scrap_model(google_id)
    return search_models(request, "")


