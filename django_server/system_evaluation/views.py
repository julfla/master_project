from django.http import HttpResponse, HttpResponseNotFound
import datetime

from django.shortcuts import render_to_response, get_object_or_404, redirect
from django.template import RequestContext

from sketchup_models.models import SketchupModel
from pointcloud.models import PointCloud
from system_evaluation.models import ExampleManager, IdentificationAttempt, EvaluationSession
from warehouse_scrapper.models import WarehouseScrapper
from identifier.models import Identifier
from system_evaluation.forms import *

def identification_result(request, identification_attempt_id="5337c2b1a533a32d6ecbd809"):
    attempt = IdentificationAttempt.objects.get(pk=identification_attempt_id)
    if attempt.identification_succeed:
        print "attempt {} succeed !".format(attempt.pk)
        if request.method == 'POST':
            form = AgreeWithIdentificationForm(request.POST)
            if form.is_valid():
                attempt.user_argreed = form.cleaned_data['user_argreed']
                attempt.user_identification = form.cleaned_data['user_identification']
                attempt.save()
                return redirect('/system/session/{}/attempt/new'.format(attempt.evaluation_session_id) )
        else:
            form = AgreeWithIdentificationForm()
        return render_to_response('identification_succeed.html',
            {'attempt':attempt, 'form':form}, context_instance=RequestContext(request) )
    else:
        print "attempt {} failed ...".format(attempt.pk)
        keywords = request.GET.get('keywords')
        models = WarehouseScrapper.search_for_models(keywords) if keywords else []
        return render_to_response('identification_failed.html',
            {'attempt': attempt, 'models': models, 'keywords':keywords if keywords else ""}, 
            context_instance=RequestContext(request)
            )

def image(request, identification_attempt_id):
    attempt = get_object_or_404(IdentificationAttempt, pk=identification_attempt_id
    image = ExampleManager.get_image(attempt.example)
    return HttpResponse(image.read(), mimetype="image/png")

def new_session(request):
    # draw a form asking the user for information
    if request.method == 'POST':
        form = NewSessionForm(request.POST)
        if form.is_valid():
            session = EvaluationSession()
            session.user = form.cleaned_data['user']
            session.save()
            return redirect('{}/attempt/new'.format(session.pk) )
    else:
        form = NewSessionForm()
    return render_to_response('new_session.html', 
        {'form':form},
        context_instance=RequestContext(request)
        )

def new_attempt(request, evaluation_session_id):
    session = get_object_or_404(EvaluationSession, pk=evaluation_session_id)
    attempt = IdentificationAttempt()
    attempt.evaluation_session = session
    attempt.example = ExampleManager.get_random_example()
    pcd_file = ExampleManager.get_pcd( attempt.example )
    print "Load {} pointcloud <{}> for identification".format(attempt.example, pcd_file.name)
    pointcloud = PointCloud.load_pcd( pcd_file.name )
    try:
        attempt.identification_result = Identifier.instance().identify( pointcloud )
        attempt.identification_succeed = True
    except:
        attempt.identification_succeed = False
    attempt.save()
    return redirect( '/system/attempt/{}/result/'.format(attempt.pk) )

def train_identifier(request):
    category = request.GET['category']
    evaluation_session_id = request.GET['evaluation_session_id']
    session = get_object_or_404(EvaluationSession, pk=evaluation_session_id)
    models = []
    for google_id in request.GET['google_ids'].split(","):
        models.append( SketchupModel.find_google_id(google_id) )

    # TODO : train !!!!!
    Identifier.instance().train(models, category)
    # then redirect to a new attempt
    print "category: {}\nmodels: {}".format(category, models.__str__())

    return redirect('/system/session/{}/attempt/new'.format(session.pk) )