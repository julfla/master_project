from django.conf.urls import patterns, include, url

from django.contrib import admin
admin.autodiscover()

urlpatterns = patterns('',
    # Examples:
    # url(r'^$', 'django_server.views.home', name='home'),
    # url(r'^blog/', include('blog.urls')),

    url(r'^admin/', include(admin.site.urls)),
    url(r'^model/(.+)/image/', 'testapp.views.model_image'),
    url(r'^model/(.+)/mesh/', 'testapp.views.model_mesh'),
    url(r'^model/(.+)/', 'testapp.views.model')
)
