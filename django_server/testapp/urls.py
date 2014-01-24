from django.conf.urls import patterns, include, url

urlpatterns = patterns('',
    url(r'^(.+)/image/', 'testapp.views.model_image'),
    url(r'^(.+)/mesh/', 'testapp.views.model_mesh'),
    url(r'^scrap/(.+)/', 'testapp.views.scrap_model'),
    url(r'^(.+)/', 'testapp.views.search_models')
)
