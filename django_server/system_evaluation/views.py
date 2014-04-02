from django.http import HttpResponse, HttpResponseNotFound
import datetime

from django.shortcuts import render_to_response
from django.template import RequestContext

from pointcloud.models import PointCloud
from system_evaluation.models import ExampleManager, IdentificationAttempt, EvaluationSession
from warehouse_scrapper.models import WarehouseScrapper
from identifier.models import Identifier

def identification_result(request, identification_attempt_id="5337c2b1a533a32d6ecbd809"):
    attempt = IdentificationAttempt.objects.get(pk=identification_attempt_id)
    pointcloud = PointCloud.load_pcd( "pointcloud/fixtures/cloud.pcd" )
    try:
        attempt.identification_result = Identifier.instance().identify( pointcloud )
        attempt.identification_succeed = True
    except:
        attempt.identification_succeed = False

    if attempt.identification_succeed:
        return render_to_response('identification_succeed.html',
            {'attempt':attempt},
            context_instance=RequestContext(request)
            )
    else:
        keywords = request.GET.get('keywords')
        models = WarehouseScrapper.search_for_models(keywords) if keywords else []
        return render_to_response('identification_failed.html',
            {'attempt': attempt, 'models': models, 'keywords':keywords if keywords else ""}, 
            context_instance=RequestContext(request)
            )

def image(request, identification_attempt_id):
    try:
        attempt = IdentificationAttempt.objects.get(pk=identification_attempt_id)
        image = ExampleManager.get_image(attempt.img_path)
        return HttpResponse(image.read(), mimetype="image/png")
    except IdentificationAttempt.DoesNotExist:
        return HttpResponseNotFound()