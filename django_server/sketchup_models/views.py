from django.http import HttpResponse, HttpResponseNotFound

from django.shortcuts import render_to_response
from django.template import RequestContext

from sketchup_models.models import SketchupModel
from warehouse_scrapper.models import search_by_keywords

def index(request):
    keywords = request.GET.get('keywords')
    print keywords
    if keywords is None:
        models = []
    else:
        models = search_by_keywords(keywords, True)
    return render_to_response('default.html', {'models': models},
                              context_instance=RequestContext(request))

def detailed_view(request, google_id):
    model = SketchupModel.find_google_id(google_id)
    print "model {} has {} views.".format(model, model.partialview_set.count())
    if model is None:
        return HttpResponseNotFound()
    return render_to_response('model_detailed_view.html', {'model': model},
                              context_instance=RequestContext(request))


def model_mesh(request, google_id):
    try:
        model = SketchupModel.find_google_id(google_id)
        if model.mesh is None:
            return HttpResponseNotFound()
        response = HttpResponse(model.mesh, mimetype="text/plain")
        response['Content-Disposition'] = 'attachment; filename=%s.tri' % model
        return response
    except SketchupModel.DoesNotExist, SketchupModel.AttributeError:
        return HttpResponseNotFound()
