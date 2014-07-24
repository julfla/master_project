from django.conf.urls import patterns, include, url

urlpatterns = patterns('',
    url(r'^example_object/(.+)/video_sequence/(.+)/video', 'system_evaluation.views.sequence_video'),
    url(r'^session/(.+)/train', 'system_evaluation.views.train_identifier'),
	url(r'^session/new', 'system_evaluation.views.new_session'),
    url(r'^session/(.+)/end', 'system_evaluation.views.end_session'),
    url(r'^session/(.+)/attempt/new', 'system_evaluation.views.new_attempt'),
	url(r'^session/(.+)/attempt/(.+)/image/', 'system_evaluation.views.image'),
    url(r'^session/(.+)/attempt/(.+)/result/', 'system_evaluation.views.identification_result')
)
