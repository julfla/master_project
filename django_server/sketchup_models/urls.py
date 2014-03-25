from django.conf.urls import patterns, include, url

urlpatterns = patterns('',
    url(r'^(.+)/image/', 'sketchup_models.views.model_image'),
    url(r'^(.+)/mesh/', 'sketchup_models.views.model_mesh'),
    url(r'^scrap/(.+)/', 'sketchup_models.views.scrap_model'),
    url(r'^index.html', 'sketchup_models.views.index')
)
