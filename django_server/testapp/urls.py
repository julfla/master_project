from django.conf.urls import patterns, include, url

urlpatterns = patterns('',
    url(r'^(.+)/image/', 'testapp.views.model_image'),
    url(r'^(.+)/mesh/', 'testapp.views.model_mesh'),
    url(r'^(.+).html', 'testapp.views.model')
)
