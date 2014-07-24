from django.http import HttpResponse, HttpResponseNotFound
from collections import defaultdict
import operator
from urllib import urlencode

from django.shortcuts import render_to_response, get_object_or_404, redirect
from django.template import RequestContext

from sketchup_models.models import SketchupModel
# from pointcloud.models import PointCloud
from .models import IdentificationAttempt, EvaluationSession
from .forms import NewSessionForm, AgreeWithIdentificationForm

from warehouse_scrapper.models import search_by_keywords

from .models import ExampleObject, VideoSequence

# This constant store the video sequence ids that can be used in the GUI.
# In order to run the HIM with a acceptable speed, we use only the video
# sequence for which all the frames have the pointcloud preloaded.
CATEGORIES_HMI = set(['plate', 'apple', 'soda_can', 'toothbrush', 'banana'])
VIDEO_SEQUENCE_HMI = set(
    [sequence
     for example in ExampleObject.objects.filter(category__in=CATEGORIES_HMI)
     for sequence in example.sequences.all()])

NUMBER_ATTEMPT_SESSION = 20


def identification_result(request, evaluation_session_id, attempt_index):
    """ Display the result of a identification attempt. """
    session = get_object_or_404(EvaluationSession, pk=evaluation_session_id)
    attempt = session.attempts[int(attempt_index)]
    if attempt.identification_succeed and attempt.user_agreed:
        print "attempt {} succeed !".format(attempt_index)
        return identication_succeeded_result(request, session,
                                             attempt, attempt_index)
    else:
        print "attempt {} failed ...".format(attempt_index)
        return identication_failed_result(request, session, attempt,
                                          attempt_index)


def identication_succeeded_result(request, session, attempt, attempt_index):
    """ Result view in the case of a successful identification. """
    if request.method == 'POST':
        print "POST : ", request.POST
        form = AgreeWithIdentificationForm(request.POST)
        if form.is_valid():
            print 'cleaned_data:', form.cleaned_data
            attempt.user_agreed = form.cleaned_data['user_agreed']
            attempt.user_identification = request.POST['user_identification']
            if attempt.user_identification not in session.identifier.categories:
                attempt.new_category_learned = True
            session.save()
            if attempt.user_agreed:
                return redirect(
                    '/system/session/{}/attempt/new'.format(session.pk))
            else:
                return redirect(
                    '/system/session/{}/attempt/{}/result/?{}'.format(
                        session.pk, attempt_index,
                        urlencode({'keywords': attempt.user_identification})))
    else:
        form = AgreeWithIdentificationForm()
    print 'render_to_response'
    return render_to_response(
        'identification_succeed.html',
        {'attempt': attempt, 'attempt_index': attempt_index,
         'session_id': session.pk, 'form': form},
        context_instance=RequestContext(request))


def identication_failed_result(request, session, attempt, attempt_index):
    """ Result view in the case of a failed identification. """
    keywords = request.GET.get('keywords', '')
    model_ids = search_by_keywords(keywords) if keywords else []
    print len(model_ids), ' models found for keywords ', keywords
    models = [SketchupModel.find_google_id(id) for id in model_ids]
    return render_to_response(
        'identification_failed.html',
        {'attempt': attempt, 'attempt_index': attempt_index,
         'session_id': session.pk, 'models': models, 'keywords': keywords},
        context_instance=RequestContext(request))


def image(request, evaluation_session_id, identification_attempt_index):
    session = get_object_or_404(EvaluationSession, pk=evaluation_session_id)
    attempt = session.attempts[int(identification_attempt_index)]
    return HttpResponse(attempt.video_sequence.video.read(),
                        mimetype="image/gif")


def new_session(request):
    """ A form view that lead to the begining a an evaluation session. """
    # DEBUG
    EvaluationSession.objects.all().delete()
    # END DEBUG
    if request.method == 'POST':
        form = NewSessionForm(request.POST)
        if form.is_valid():
            session = EvaluationSession()
            session.user = form.cleaned_data['user']
            session.save()
            return redirect('{}/attempt/new'.format(session.pk))
    else:
        form = NewSessionForm()
    return render_to_response('new_session.html',
                              {'form': form},
                              context_instance=RequestContext(request))


def end_session(request, evaluation_session_id):
    """ Display the result of the evaluation session. """
    session = get_object_or_404(EvaluationSession, pk=evaluation_session_id)
    failures = [attempt for attempt in session.attempts
                if not attempt.user_agreed]
    objects_learned = [attempt for attempt in session.attempts
                       if attempt.new_category_learned]
    return render_to_response(
        'end_of_session.html',
        {'session': session,
         'number_failures': len(failures) - len(objects_learned),
         'number_objects_learned': len(objects_learned),
         'number_attempts': len(session.attempts)},
        context_instance=RequestContext(request))


def new_attempt(request, evaluation_session_id):
    """ Create an identification_attempt and try to identify. """
    session = get_object_or_404(EvaluationSession, pk=evaluation_session_id)
    if len(session.attempts) == NUMBER_ATTEMPT_SESSION:
        # The evaluation is complete
        # We redirect to its end page instead of continuing.
        return redirect('/system/session/{}/end'.format(session.pk))

    attempt = IdentificationAttempt()
    session.attempts.append(attempt)
    import random
    video_sequence = random.sample(VIDEO_SEQUENCE_HMI, 1)[0]
    attempt.video_sequence = video_sequence
    print "Identifying {}_{}...".format(video_sequence.example_object.name,
                                        video_sequence.sequence_id)
    try:
        results = defaultdict(int)
        for frame in video_sequence.frames.iterator():
            result = session.identifier.identify(frame.pointcloud)
            results[result] += 1
        sorted_results = sorted(results.iteritems(), reverse=True,
                                key=operator.itemgetter(1))
        attempt.identification_result = sorted_results[0][0]
        attempt.identification_succeed = True
        print "    Indentified as a ", attempt.identification_result
    except:  # TODO PRECISE EXCEPTION !!!
        attempt.identification_succeed = False
        print "    Indentification failed..."
    session.save()
    return redirect('/system/session/{}/attempt/{}/result/'.format(
        session.pk, len(session.attempts) - 1))


def train_identifier(request, evaluation_session_id):
    """ Train the identifier then redirect to a new attempt. """
    session = get_object_or_404(EvaluationSession, pk=evaluation_session_id)
    category = request.GET['category']
    model_ids = request.GET['google_ids'].split(",")
    models = [SketchupModel.find_google_id(id) for id in model_ids]
    session.identifier.add_models(models, category)
    session.identifier.train()
    session.save()
    return redirect('/system/session/{}/attempt/new'.format(session.pk))


def sequence_video(request, object_name, sequence_id):
    """ Render the gif image corresponding the a VideoSequence. """
    example = get_object_or_404(ExampleObject, name=object_name)
    sequence = get_object_or_404(VideoSequence, example_object=example,
                                 sequence_id=sequence_id)
    return HttpResponse(sequence.video.read(), mimetype="image/gif")
