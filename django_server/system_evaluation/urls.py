from django.conf.urls import patterns, include, url

urlpatterns = patterns('',
	url(r'^(.+)/image/', 'system_evaluation.views.image'),
    url(r'^', 'system_evaluation.views.identification_result')
)
