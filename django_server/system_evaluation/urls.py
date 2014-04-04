from django.conf.urls import patterns, include, url

urlpatterns = patterns('',
    url(r'^train', 'system_evaluation.views.train_identifier'),
	url(r'^session/new', 'system_evaluation.views.new_session'),
	url(r'^session/(.+)/attempt/new', 'system_evaluation.views.new_attempt'),
	url(r'^attempt/(.+)/image/', 'system_evaluation.views.image'),
    url(r'^attempt/(.+)/result/', 'system_evaluation.views.identification_result')
)
