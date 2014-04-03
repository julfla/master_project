from django.conf.urls import patterns, include, url

urlpatterns = patterns('',
	url(r'^(.+)/image/', 'system_evaluation.views.image'),
    url(r'^(.+)/train/', 'system_evaluation.views.train_identifier'),
    url(r'^(.+)/result/', 'system_evaluation.views.identification_result')
)
