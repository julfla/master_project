from django.http import HttpResponse, HttpResponseNotFound
import datetime

from django.shortcuts import render_to_response, get_object_or_404, redirect
from django.template import RequestContext

# from sketchup_models.models import SketchupModel
# from pointcloud.models import PointCloud
# from system_evaluation.models import Example, IdentificationAttempt, EvaluationSession
# from warehouse_scrapper.models import WarehouseScrapper
# from system_evaluation.forms import *

from .models import ExampleObject, VideoSequence, Frame

# def identification_result(request, evaluation_session_id, identification_attempt_index):
#     session = get_object_or_404(EvaluationSession, pk=evaluation_session_id)
#     attempt = session.attempts[int(identification_attempt_index)]
#     if attempt.identification_succeed:
#         print "attempt {} succeed !".format(identification_attempt_index)
#         if request.method == 'POST':
#             form = AgreeWithIdentificationForm(request.POST)
#             if form.is_valid():
#                 attempt.user_agreed = form.cleaned_data['user_agreed']
#                 attempt.user_identification = form.cleaned_data['user_identification']
#                 session.save()
#                 return redirect('/system/session/{}/attempt/new'.format(evaluation_session_id) )
#         else:
#             form = AgreeWithIdentificationForm()
#         return render_to_response('identification_succeed.html',
#             {'attempt':attempt, 'attempt_index':identification_attempt_index,
#              'session_id':session.pk, 'form':form}, context_instance=RequestContext(request) )
#     else:
#         print "attempt {} failed ...".format(identification_attempt_index)
#         keywords = request.GET.get('keywords')
#         models = WarehouseScrapper.search_for_models(keywords) if keywords else []
#         return render_to_response('identification_failed.html',
#             {'attempt': attempt, 'attempt_index':identification_attempt_index,
#              'session_id':session.pk,'models': models, 'keywords':keywords if keywords else ""},
#             context_instance=RequestContext(request)
#             )

# def image(request, evaluation_session_id, identification_attempt_index):
#     session = get_object_or_404(EvaluationSession, pk=evaluation_session_id)
#     attempt = session.attempts[int(identification_attempt_index)]
#     image = Example.objects.get(name=attempt.example).image_file()
#     return HttpResponse(image.read(), mimetype="image/png")

# def new_session(request):
#     # draw a form asking the user for information
#     if request.method == 'POST':
#         form = NewSessionForm(request.POST)
#         if form.is_valid():
#             session = EvaluationSession()
#             session.user = form.cleaned_data['user']
#             session.save()
#             return redirect('{}/attempt/new'.format(session.pk) )
#     else:
#         form = NewSessionForm()
#     return render_to_response('new_session.html',
#         {'form':form},
#         context_instance=RequestContext(request)
#         )

# def new_attempt(request, evaluation_session_id):
#     session = get_object_or_404(EvaluationSession, pk=evaluation_session_id)
#     attempt = IdentificationAttempt()
#     example = Example.get_random()
#     attempt.example = example.name
#     pcd_file = example.pcd_file()
#     print "Load {} pointcloud <{}> for identification".format(attempt.example, pcd_file.name)
#     pointcloud = PointCloud.load_pcd( pcd_file.name )
#     try:
#         attempt.identification_result = session.identifier.identify( pointcloud )
#         attempt.identification_succeed = True
#     except:
#         attempt.identification_succeed = False
#     session.attempts.append( attempt )
#     session.save()
#     return redirect( '/system/session/{}/attempt/{}/result/'.format(
#         session.pk, len(session.attempts) - 1)
#     )

# def train_identifier(request, evaluation_session_id):
#     session = get_object_or_404(EvaluationSession, pk=evaluation_session_id)
#     category = request.GET['category']
#     models = []
#     for google_id in request.GET['google_ids'].split(","):
#         models.append( SketchupModel.find_google_id(google_id) )

#     # TODO : train !!!!!
#     session.identifier.add_models(models, category)
#     session.identifier.train()
#     session.identifier.save()
#     # then redirect to a new attempt
#     print "category: {}\nmodels: {}".format(category, models.__str__())

#     return redirect('/system/session/{}/attempt/new'.format(session.pk) )

def sequence_video(request, object_name, sequence_id):
    example = get_object_or_404(ExampleObject, name=object_name)
    sequence = get_object_or_404(VideoSequence, example_object=example,
                                 sequence_id=sequence_id)
    return HttpResponse(sequence.video.read(), mimetype="image/gif")
