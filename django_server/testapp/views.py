from django.http import HttpResponse, HttpResponseNotFound
import datetime

from testapp.models import SketchupModel

def model(request, google_id):
    now = datetime.datetime.now()
    try:
        model = SketchupModel.objects.get(google_id=google_id)
        html = "<html><body>It is now {0}. Model id is {1}.</body></html>".format(now, model)
    except SketchupModel.DoesNotExist:
        html = "<html><body>It is now {0}. Model id doesn't exist.</body></html>".format(now)
    return HttpResponse(html)

def model_image(request, google_id):
    try:
        model = SketchupModel.objects.get(google_id=google_id)
        return HttpResponse(model.image.read(), mimetype="image/png")
    except SketchupModel.DoesNotExist:
        return HttpResponseNotFound("<h1>Page not found</h1>")

def model_mesh(request, google_id):
    try:
        model = SketchupModel.objects.get(google_id=google_id)
        return HttpResponse(model.mesh.read(), mimetype="text/plain")
    except SketchupModel.DoesNotExist:
        return HttpResponseNotFound("<h1>Page not found</h1>")

