from django.conf.urls import patterns, include, url

urlpatterns = patterns('',
	url(r'^(.+)/detailed_view', 'sketchup_models.views.detailed_view'),
    url(r'^(.+)/mesh/', 'sketchup_models.views.model_mesh'),
    url(r'^index.html', 'sketchup_models.views.index')
)
