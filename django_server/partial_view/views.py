from django.http import HttpResponse, HttpResponseNotFound

from django.shortcuts import render_to_response
from django.template import RequestContext

from sketchup_models.models import SketchupModel
from partial_view.models import PartialView
from common.libs.libpypartialview import PointCloud, PartialViewComputer

import tempfile

def view_pcd(request, google_id):
    try:
    	print "Request pcd"
        model = SketchupModel.find_google_id(google_id)
        print "Init View"
        view = PartialView()
        view.compute_view(model, 0.0, 0.0)
        print "View Initialized"
        f = tempfile.NamedTemporaryFile()
        view.pointcloud.save_pcd(f.name)
        rep = HttpResponse(f.read(), mimetype="text/plain")
        view.save()
        return rep
    except SketchupModel.DoesNotExist:
        return HttpResponseNotFound()


